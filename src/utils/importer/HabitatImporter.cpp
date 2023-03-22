// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/ArrayTuple.h>
#include <Corrade/Containers/BitArray.h>
#include <Corrade/Containers/BitArrayView.h>
#include <Corrade/Containers/StridedBitArrayView.h>
#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/Format.h>
#include <Corrade/Utility/Json.h>
#include <Corrade/Utility/Path.h>
#include <Magnum/Math/Vector4.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Swizzle.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/SceneData.h>

namespace Cr = Corrade;
namespace Mn = Magnum;
using namespace Cr::Containers::Literals;
using namespace Mn::Math::Literals;

namespace {

struct ImporterState {
  Cr::Containers::String basePath;
  Cr::Utility::Json json;
  const Cr::Utility::JsonToken& jsonStage;
  Cr::Containers::Array<Cr::Containers::Reference<const Cr::Utility::JsonToken>> jsonObjects;
};

class HabitatImporter: public Mn::Trade::AbstractImporter {
  public:
    explicit HabitatImporter(Cr::PluginManager::AbstractManager& manager, const Cr::Containers::StringView plugin): Mn::Trade::AbstractImporter{manager, plugin} {}

  private:
    Mn::Trade::ImporterFeatures doFeatures() const override { return {}; }
    bool doIsOpened() const override { return !!_state; }
    void doClose() override { _state = Cr::Containers::NullOpt; }
    void doOpenFile(Cr::Containers::StringView filename) override;

    Mn::UnsignedInt doSceneCount() const override { return 1; }
    Cr::Containers::Optional<Mn::Trade::SceneData> doScene(Mn::UnsignedInt id) override;
    Mn::Trade::SceneField doSceneFieldForName(Cr::Containers::StringView name) override;
    Cr::Containers::String doSceneFieldName(Mn::UnsignedInt name) override;

    Mn::UnsignedLong doObjectCount() const override;
    Mn::Long doObjectForName(Cr::Containers::StringView name) override;
    Cr::Containers::String doObjectName(Mn::UnsignedLong id) override;

    Cr::Containers::Optional<ImporterState> _state;
};

void HabitatImporter::doOpenFile(Cr::Containers::StringView filename) {
  Cr::Containers::Optional<Cr::Utility::Json> json = Cr::Utility::Json::fromFile(filename, Cr::Utility::Json::Option::ParseFloats|Cr::Utility::Json::Option::ParseLiterals|Cr::Utility::Json::Option::ParseStrings);
  if(!json) {
    Mn::Error{} << "HabitatImporter::openFile(): can't open the top-level JSON";
    return;
  }

  /* Gather references to all stages and objects in the JSON */
  // TODO support also loading the scene/object JSONs directly?
  const Cr::Utility::JsonToken& stage = json->root()["stage_instance"_s];
  Cr::Containers::Array<Cr::Containers::Reference<const Cr::Utility::JsonToken>> objects;
  for(const Cr::Utility::JsonToken& object: json->root()["object_instances"_s].asArray()) {
    arrayAppend(objects, object);
  }

  _state.emplace(
    Cr::Utility::Path::split(filename).first(),
    *std::move(json),
    stage,
    std::move(objects));
}

/* Underscore to have them start with a lowercase */
constexpr Mn::Trade::SceneField SceneField_externalFile = Mn::Trade::sceneFieldCustom(0);
constexpr Mn::Trade::SceneField SceneField_semanticId = Mn::Trade::sceneFieldCustom(1);
constexpr Mn::Trade::SceneField SceneField_up = Mn::Trade::sceneFieldCustom(2);
constexpr Mn::Trade::SceneField SceneField_front = Mn::Trade::sceneFieldCustom(3);
constexpr Mn::Trade::SceneField SceneField_requiresLighting = Mn::Trade::sceneFieldCustom(4);

Cr::Containers::String HabitatImporter::doSceneFieldName(const Mn::UnsignedInt name) {
  switch(name) {
    #define _c(field) \
      case Mn::Trade::sceneFieldCustom(SceneField_ ## field): return #field;
    _c(externalFile)
    _c(semanticId)
    _c(up)
    _c(front)
    _c(requiresLighting)
    #undef _c
  }

  return {};
}

Mn::Trade::SceneField HabitatImporter::doSceneFieldForName(const Cr::Containers::StringView name) {
  #define _c(field) \
    if(name == #field) return SceneField_ ## field;
  _c(externalFile)
  _c(semanticId)
  _c(up)
  _c(front)
  _c(requiresLighting)
  #undef _c

  return {};
}

Cr::Containers::Optional<Mn::Trade::SceneData> HabitatImporter::doScene(Mn::UnsignedInt) {
  struct Object {
    Mn::UnsignedInt mapping;
    Mn::Vector3 translation;
    Mn::Quaternion rotation;
    Mn::Vector3 scaling;
    Mn::Vector3 up;
    Mn::Vector3 front;
  };
  Cr::Containers::Array<Object> objects{Cr::NoInit, _state->jsonObjects.size()};

  struct SemanticId {
    Mn::UnsignedInt mapping;
    Mn::UnsignedInt semanticId;
  };
  Cr::Containers::Array<SemanticId> semanticIds;

  struct ExternalFile {
    Mn::UnsignedInt mapping;
    Mn::UnsignedInt stringOffset;
  };
  Cr::Containers::Array<ExternalFile> externalFiles{Cr::NoInit, _state->jsonObjects.size() + 1};
  Cr::Containers::Array<char> externalFileStrings;
  /* This shares the mapping with externalFiles */
  Cr::Containers::BitArray requiresLighting{Cr::NoInit, _state->jsonObjects.size() + 1};

  // TODO there's `"translation_origin": "asset_local"` specified for the whole
  //  file, what to do with it? why there has to be several ways to achieve the
  //  same?!

  /* Parse the stage file, which is object 0 */
  {
    Cr::Containers::Pair<Cr::Containers::StringView, Cr::Containers::StringView> nestedPathName = Cr::Utility::Path::split(_state->jsonStage["template_name"_s].asString());
    Cr::Containers::String nestedPath = Cr::Utility::Path::join(".."_s, nestedPathName.first());
    Cr::Containers::String nestedFilename = Cr::Utility::Path::join({_state->basePath, nestedPath, nestedPathName.second() + ".stage_config.json"_s});

    /* Parse the nested JSON */
    const Cr::Containers::Optional<Cr::Utility::Json> jsonStageNested = Cr::Utility::Json::fromFile(nestedFilename, Cr::Utility::Json::Option::ParseFloats|Cr::Utility::Json::Option::ParseLiterals|Cr::Utility::Json::Option::ParseStrings);
    if(!jsonStageNested) {
      Mn::Error{} << "HabitatImporter::scene(): can't open the stage JSON";
      return {};
    }

    /* External file path. It's relative to the path the JSON was found in,
       convert that to be relative to the top-level JSON */
    arrayAppend(externalFileStrings, Cr::Utility::Path::join(nestedPath, jsonStageNested->root()["render_asset"_s].asString()));
    externalFiles[0].mapping = 0;
    externalFiles[0].stringOffset = externalFileStrings.size();

    /* Other attributes */
    requiresLighting.set(0, jsonStageNested->root()["requires_lighting"_s].asBool());
    // TODO margin, friction/restitution coefficient -- used only for the arch
    //  file?
  }

  /* Parse all objects */
  for(std::size_t i = 0; i != _state->jsonObjects.size(); ++i) {
    const Mn::UnsignedInt objectId = i + 1;
    const Cr::Utility::JsonToken& jsonObject = _state->jsonObjects[i];
    const Cr::Containers::StringView templateName = jsonObject["template_name"_s].asString();

    /* There's a ... uh ... few possible cases where the nested JSONs could be.
       Some paths are listed in the top-level `*.scene_dataset_config.json` but
       it suffers from a similar problem -- it's either in ../../ or in ../,
       and it doesn't have a fixed name so I'd have to glob for it.

       And even that doesn't help, as the nested JSON could be also in any
       arbitrary subdirectory. For now, to get things done, I'm just hardcoding
       some possibilities, this needs to be cleaned up. */
    // TODO clean up this mess in the datasets themselves!!
    Cr::Containers::String nestedPath;
    Cr::Containers::String nestedFilename;
    for(Cr::Containers::String path: {
      /* In the 0.2.0 version of FP the files were directly in root and the
         template_name contained the objects/ or scenes/ prefix */
      Cr::Containers::String::nullTerminatedGlobalView(".."_s),
      /* In the HF main the template_name is just the hash, so we need to
         prepend objects/ to it, and additionally, because there are too many,
         they're put into subdirectories based on the first letter of the hash
         (so 1efdc3d37dfab1eb9f99117bb84c59003d684811 is in
         objects/1/1efdc3d37dfab1eb9f99117bb84c59003d684811.object_config.json) */
      Cr::Utility::Path::join("../objects"_s, templateName.prefix(1)),
      /* Some objects are also inside openings/ */
      Cr::Containers::String::nullTerminatedGlobalView("../objects/openings"_s),
      /* And some in decomposed/ with additional subdirectories named after the
         template base hash */
      Cr::Utility::Path::join("../objects/decomposed"_s, templateName.prefix(Mn::Math::min(std::size_t{40}, templateName.size()))),
    }) {
      const Cr::Containers::String filename = Cr::Utility::Path::join({_state->basePath, path, templateName + ".object_config.json"_s});
      if(Cr::Utility::Path::exists(filename)) {
        nestedPath = path;
        nestedFilename = filename;
        break;
      }
    }

    if(!nestedFilename) {
      Mn::Error{} << "HabitatImporter::scene(): can't find object JSON file for" << templateName;
      return {};
    }

    /* Parse the nested JSON */
    Cr::Containers::Optional<Cr::Utility::Json> jsonObjectNested = Cr::Utility::Json::fromFile(nestedFilename, Cr::Utility::Json::Option::ParseLiterals|Cr::Utility::Json::Option::ParseFloats|Cr::Utility::Json::Option::ParseStrings);
    if(!jsonObjectNested) {
      Mn::Error{} << "HabitatImporter::scene(): can't open the object JSON";
      return {};
    }

    /* External file path. It's relative to the path the JSON was found in,
       convert that to be relative to the top-level JSON */
    arrayAppend(externalFileStrings, Cr::Utility::Path::join(nestedPath, jsonObjectNested->root()["render_asset"_s].asString()));
    externalFiles[i + 1].mapping = objectId;
    externalFiles[i + 1].stringOffset = externalFileStrings.size();

    /* Other attributes. Some stored in the top-level JSON, some in the
       nested. */
    objects[i].mapping = objectId;
    Cr::Utility::copy(jsonObject["translation"_s].asFloatArray(3),
                      objects[i].translation.data());
    Cr::Utility::copy(jsonObject["rotation"_s].asFloatArray(4),
                      objects[i].rotation.data());
    // TODO "uniform_scale"?! why is there two things for doing the same, and
    //  non_uniform_scale being usually [1, 1, 1]?!
    Cr::Utility::copy(jsonObject["non_uniform_scale"_s].asFloatArray(3),
                      objects[i].scaling.data());
    // TODO are these ever set to anything else than Y up and Z backward? like,
    //  why not have them a part of the rotation matrix?!
    Cr::Utility::copy(jsonObjectNested->root()["up"_s].asFloatArray(3),
                      objects[i].up.data());
    Cr::Utility::copy(jsonObjectNested->root()["front"_s].asFloatArray(3),
                      objects[i].front.data());
    requiresLighting.set(objectId, jsonObjectNested->root()["requires_lighting"_s].asBool());
    // TODO motion type (STATIC, some other?)
    // TODO rotation

    /* For some reason, the quaternion is written as WXYZ?! And additionally
       the rotation is done in some completely crazy coordinate system that has
       no relation whatsoever to the up/front vectors above, nor to the
       coordinate system that translation and scaling is done in. IMAGINE such
       a mess happening intentionally. */
    Mn::Vector4::from(objects[i].rotation.data()) =  Mn::Math::gather<'w', 'x', 'y', 'z'>(Mn::Vector4::from(objects[i].rotation.data()));
    objects[i].rotation =
      Mn::Quaternion::rotation(180.0_degf, Mn::Vector3::yAxis())*
      // TODO rotating around x instead of z works as well, why? can't this be
      //  simplified further?
      Mn::Quaternion::rotation(-180.0_degf, Mn::Vector3::zAxis())*
      objects[i].rotation*
      Mn::Quaternion::rotation(180.0_degf, Mn::Vector3::zAxis());

    /* Semantic ID. Present only for furniture etc., not the openings */
    if(const Cr::Utility::JsonToken* jsonObjectNestedSemanticId = jsonObjectNested->root().find("semantic_id"_s)) {
      arrayAppend(semanticIds, Cr::InPlaceInit, objectId, *jsonObjectNested->parseUnsignedInt(*jsonObjectNestedSemanticId));
    }
  }

  Cr::Containers::StridedArrayView1D<Object> outputObjects;
  Cr::Containers::StridedArrayView1D<SemanticId> outputSemanticIds;
  Cr::Containers::StridedArrayView1D<ExternalFile> outputExternalFiles;
  Cr::Containers::MutableStringView outputExternalFileStrings;
  Cr::Containers::MutableBitArrayView outputRequiresLighting;
  Cr::Containers::Array<char> data = Cr::Containers::ArrayTuple{
    {Cr::NoInit, objects.size(), outputObjects},
    {Cr::NoInit, semanticIds.size(), outputSemanticIds},
    {Cr::NoInit, externalFiles.size(), outputExternalFiles},
    {Cr::NoInit, externalFileStrings.size(), outputExternalFileStrings},
    {Cr::NoInit, requiresLighting.size(), outputRequiresLighting}
  };
  // TODO have some DirectInit instead or, better, have SceneTools::combine()
  Cr::Utility::copy(objects, outputObjects);
  Cr::Utility::copy(semanticIds, outputSemanticIds);
  Cr::Utility::copy(externalFiles, outputExternalFiles);
  Cr::Utility::copy(externalFileStrings, outputExternalFileStrings);
  // TODO have a copy() for bit arrays, finally
  Cr::Utility::copy(
    Cr::Containers::arrayView(requiresLighting.data(), (requiresLighting.size() + 7)/8),
    Cr::Containers::arrayView(static_cast<char*>(outputRequiresLighting.data()), (outputRequiresLighting.size() + 7)/8));

  return Mn::Trade::SceneData{Mn::Trade::SceneMappingType::UnsignedInt, _state->jsonObjects.size() + 1, std::move(data), {
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Translation,
      outputObjects.slice(&Object::mapping),
      outputObjects.slice(&Object::translation),
      Mn::Trade::SceneFieldFlag::OrderedMapping},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Rotation,
      outputObjects.slice(&Object::mapping),
      outputObjects.slice(&Object::rotation),
      Mn::Trade::SceneFieldFlag::OrderedMapping},
    Mn::Trade::SceneFieldData{Mn::Trade::SceneField::Scaling,
      outputObjects.slice(&Object::mapping),
      outputObjects.slice(&Object::scaling),
      Mn::Trade::SceneFieldFlag::OrderedMapping},
    Mn::Trade::SceneFieldData{SceneField_up,
      outputObjects.slice(&Object::mapping),
      outputObjects.slice(&Object::up),
      Mn::Trade::SceneFieldFlag::OrderedMapping},
    Mn::Trade::SceneFieldData{SceneField_front,
      outputObjects.slice(&Object::mapping),
      outputObjects.slice(&Object::front),
      Mn::Trade::SceneFieldFlag::OrderedMapping},
    Mn::Trade::SceneFieldData{SceneField_semanticId,
      outputSemanticIds.slice(&SemanticId::mapping),
      outputSemanticIds.slice(&SemanticId::semanticId),
      Mn::Trade::SceneFieldFlag::OrderedMapping},
    Mn::Trade::SceneFieldData{SceneField_externalFile,
      outputExternalFiles.slice(&ExternalFile::mapping),
      outputExternalFileStrings.data(), Mn::Trade::SceneFieldType::StringOffset32,
      outputExternalFiles.slice(&ExternalFile::stringOffset),
      Mn::Trade::SceneFieldFlag::ImplicitMapping},
    Mn::Trade::SceneFieldData{SceneField_requiresLighting,
      outputExternalFiles.slice(&ExternalFile::mapping),
      outputRequiresLighting,
      Mn::Trade::SceneFieldFlag::ImplicitMapping},
  }};
}

Mn::UnsignedLong HabitatImporter::doObjectCount() const {
  return _state->jsonObjects.size() + 1;
}

Cr::Containers::String HabitatImporter::doObjectName(Mn::UnsignedLong id) {
  // TODO there's fpss/metadata/objects.json containing the actual names, use
  //  that instead? is that a "standard" location?

  if(id == 0)
    return _state->jsonStage["template_name"_s].asString().exceptPrefix("stages/"_s);

  const Cr::Containers::StringView templateName = (*_state->jsonObjects[id - 1])["template_name"_s].asString();
  // TODO which of the two is correct? old (0.2.0) had a prefix as well as
  //  ReplicaCAD etc, new doesn't
  return templateName.hasPrefix("objects/"_s) ? templateName.exceptPrefix("objects/"_s) : templateName;
}

Mn::Long HabitatImporter::doObjectForName(Cr::Containers::StringView name) {
  {
    if(name == _state->jsonStage["template_name"_s].asString().exceptPrefix("stages/"_s))
      return 0;
  }

  /* Yes, linear, do not use in perf-critical code */
  for(std::size_t i = 0; i != _state->jsonObjects.size(); ++i) {
    const Cr::Utility::JsonToken& object = _state->jsonObjects[i];
    const Cr::Containers::StringView templateName = object["template_name"_s].asString();

    // TODO which of the two is correct? old (0.2.0) had a prefix as well as
    //  ReplicaCAD etc, new doesn't
    if(name == (templateName.hasPrefix("objects/"_s) ? templateName.exceptPrefix("objects/"_s) : templateName))
      return i + 1;
  }

  return -1;
}

}

CORRADE_PLUGIN_REGISTER(HabitatImporter, HabitatImporter,
    "cz.mosra.magnum.Trade.AbstractImporter/0.5")

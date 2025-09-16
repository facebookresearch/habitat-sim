import bpy
import json
import csv
from typing import Any
from pathlib import Path


class LOAD_DECOMP_OT_operator(bpy.types.Operator):
    bl_idname = "load_decomp.operator"
    bl_label = "Load decomposed object"
    bl_description = "Load a decomposed object from its JSON file, properly instancing and arranging its constituent GLBs"
    bl_options = {"REGISTER", "UNDO"}

    outer_folder: bpy.props.StringProperty(
        name="Outer Folder",
        description="Location of the decomposed object folders",
        default="/home/jseagull/dev/fp-models/objects/decomposed",
    )  # type:ignore

    uuid: bpy.props.StringProperty(
        name="Object UUID",
        description="UUID (can include '_part_X' suffix)",
        default="44a88da97b60a73257237b8bfe6e87dbfe1106c8",
    )  # type:ignore

    import_reference: bpy.props.BoolProperty(
        name="Import Reference Object",
        description="Import the non-decomposed reference object for comparison",
        default=True,
    )  # type:ignore

    clear_scene: bpy.props.BoolProperty(
        name="Clear Scene", description="Clear the scene before import", default=True
    )  # type:ignore

    import_pickables: bpy.props.BoolProperty(
        name="Import pickable objects",
        description="Include the pickable objects",
        default=True,
    )  # type:ignore

    pickable_csv: bpy.props.StringProperty(
        name="Pickable CSV",
        description="Location of the CSV containing pickable data",
        default="/home/jseagull/dev/fp-models/diagnostics/parts_census/parts_census.csv",
    )  # type:ignore

    def execute(self, context):
        self.make_decomp_from_json(
            self.uuid,
            self.outer_folder,
            self.import_reference,
            self.clear_scene,
            self.import_pickables,
            self.pickable_csv,
        )
        return {"FINISHED"}

    @staticmethod
    def purge_unused_data():
        """Remove all mesh, image, and collection data that are not used in the scene."""
        if bpy.context.mode == "EDIT_MESH":
            bpy.ops.object.mode_set(mode="OBJECT")
        bpy.ops.object.select_all(action="SELECT")
        bpy.ops.object.delete()
        for collection in list(bpy.data.collections):
            bpy.data.collections.remove(collection)
        for mesh in bpy.data.meshes:
            if not mesh.users:
                bpy.data.meshes.remove(mesh)
        for image in bpy.data.images:
            if not image.users:
                bpy.data.images.remove(image)
        print("finished clearing scene")

    @staticmethod
    def open_json(json_path: Path) -> dict[str, Any]:

        if not json_path.exists():
            raise FileNotFoundError(f"JSON file not found: {json_path}")
        with open(json_path, "r") as file:
            data = json.load(file)
        return data

    @staticmethod
    def swap_axes(data: dict[str, Any]) -> dict[str, Any]:
        transform = {
            "position": [0, 0, 0],
            "quaternion": [1, 0, 0, 0],
            "scale": [1, 1, 1],
        }
        if "position" in data:
            position = data["position"]
            transform["position"] = [position[0], -position[2], position[1]]
        if "quaternion" in data:
            quaternion = data["quaternion"]
            # blender wants WXYZ in Z-up, Habitat JSON is Y-up as XYZW, so we swap a bunch of axes here
            transform["quaternion"] = [
                quaternion[3],
                quaternion[0],
                quaternion[2],
                quaternion[1],
            ]
        if "scale" in data:
            scale = data["scale"]
            transform["scale"] = [scale[0], scale[2], scale[1]]
        else:
            print("No transform found in data, using default.")
        return transform

    def place_object(self, obj: bpy.types.Object, transform: dict[str, Any]) -> None:
        """Place the object in the scene based on the provided transform."""
        obj.location = transform.get("position", (0, 0, 0))
        obj.rotation_quaternion = transform.get("quaternion", (1, 0, 0, 0))
        obj.scale = transform.get("scale", (1, 1, 1))

    def make_collection(
        self, name: str, color: str, parent: bpy.types.Collection
    ) -> bpy.types.Collection:
        if name in bpy.data.collections:
            return bpy.data.collections[name]
        new_collection = bpy.data.collections.new(name)
        new_collection.color_tag = (
            color  # this is an enum from bpy.types import ColorTag
        )
        bpy.context.scene.collection.children.link(
            new_collection
        )  # Link to the scene collection
        # If a parent is specified, link to it as well
        bpy.context.scene.collection.children.unlink(new_collection)
        parent.children.link(new_collection)
        return new_collection

    def relink_object_to_collection(
        self,
        obj: list[bpy.types.Object],
        collection: bpy.types.Collection,
        remove_from_root: bool = True,
    ) -> None:
        """Relink an object to a specified collection."""
        for o in obj:
            if remove_from_root:
                try:
                    bpy.context.scene.collection.objects.unlink(o)
                except:
                    pass
            collection.objects.link(o)  # Link to the new collection

    def make_decomp_from_json(
        self,
        uuid: str,
        outer_folder: str,
        import_reference: bool,
        clear_scene: bool,
        import_pickables: bool,
        pickable_csv: str,
    ) -> None:
        """Create a Blender scene from a decomposed JSON file."""
        # QOL - accept either uuid or part tag from spreadsheet
        uuid = uuid.split("_")[0]
        json_path = Path(f"{outer_folder}/{uuid}/{uuid}.parts.metadata.json")
        data = self.open_json(json_path)
        part_root = json_path.parents[0]
        file_prefix = json_path.name.split(".")[0]
        part_prefix = f"{file_prefix}_part_"
        prefix = data.get("id", "")
        pickable_data = {}

        def is_pickable(part_Id: str) -> bool:
            part_uuid = f"{uuid}_part_{part_Id}"
            return pickable_data[part_uuid]

        if clear_scene:
            self.purge_unused_data()

        parent_collection = self.make_collection(
            file_prefix, "NONE", bpy.context.scene.collection
        )

        # Import dedup file for comparison
        if import_reference:
            dedup_collection = self.make_collection(
                file_prefix + "_dedup", "COLOR_05", bpy.context.scene.collection
            )
            existing_objs = set(bpy.context.scene.objects)
            dedup_file = part_root / f"{file_prefix}_dedup.glb"
            if not dedup_file.exists():
                raise FileNotFoundError(f"Dedup file not found: {dedup_file}")
            bpy.ops.import_scene.gltf(filepath=str(dedup_file))
            new_objs = set(bpy.context.scene.objects) - existing_objs
            self.relink_object_to_collection(new_objs, dedup_collection)

        # Set up part list
        obj_data = data.get("parts", {})

        # Remove pickables if needed

        if not import_pickables:
            with open(pickable_csv, "r", encoding="utf-8") as csv_file:
                reader = csv.reader(csv_file)
                next(reader,None)
                for row in reader:
                    key = row[0]
                    value = row[3] # encoded as string Yes/No values
                    if value == 'Yes':
                        value = True
                    elif value == 'No':
                        value = False
                    pickable_data[key] = value
        # dbug
        print(f"Pickables: {pickable_data}")

        # Import and place unique objects
        for obj in obj_data:
            if obj.get("isRef") is not False:  # True or None - actual mesh
                part_Id = obj.get("partId", "")
                if not import_pickables and is_pickable(part_Id):
                    continue
                transform_dict = self.swap_axes(obj.get("transform", {}))
                existing_objs = set(bpy.context.scene.objects)
                part_file = part_root / f"{part_prefix}{part_Id}.glb"

                bpy.ops.import_scene.gltf(filepath=str(part_file))
                new_objs = set(bpy.context.scene.objects) - existing_objs
                part_collection = self.make_collection(
                    f"part_{part_Id}", "COLOR_04", parent_collection
                )
                self.relink_object_to_collection(new_objs, part_collection)
                for new_obj in new_objs:
                    if transform_dict:
                        self.place_object(new_obj, transform_dict)

        # Clone and place instances now that all meshes exist
        for obj in data.get("parts", {}):
            if obj.get("isRef") is False:  # These are instances
                part_Id = obj.get("partId", "")
                ref_Id = obj.get("refPartId", "")
                if not import_pickables and is_pickable(ref_Id):
                    continue
                transform_dict = self.swap_axes(obj.get("transform", {}))
                ref_collection = bpy.data.collections.get(f"part_{ref_Id}")
                if ref_collection:
                    part_collection = self.make_collection(
                        f"part_{part_Id}", "COLOR_03", parent_collection
                    )
                    for obj in ref_collection.objects:
                        new_obj = obj.copy()
                        self.relink_object_to_collection(
                            [new_obj], part_collection, False
                        )
                        if transform_dict:
                            self.place_object(new_obj, transform_dict)
                else:
                    raise ValueError(
                        f"Reference collection for part {ref_Id} not found."
                    )
        print(f"Decomposed scene created from {json_path.name} with prefix {prefix}.")


class LOAD_DECOMP_PT_panel(bpy.types.Panel):
    bl_idname = "load_decomp.panel"
    bl_label = "Load Decomp"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Load Decomp"

    def draw(self, context):
        layout = self.layout
        # Create a button that calls the operator
        op = layout.operator("load_decomp.operator", text="Load Decomposed Model")
        # This creates the UI controls that set the operator's parameters
        layout.prop(
            context.window_manager.operator_properties_last("LOAD_DECOMP_OT_operator"),
            "uuid",
        )
        layout.prop(
            context.window_manager.operator_properties_last("LOAD_DECOMP_OT_operator"),
            "outer_folder",
        )
        row = layout.row()
        row.prop(
            context.window_manager.operator_properties_last("LOAD_DECOMP_OT_operator"),
            "import_reference",
        )
        row.prop(
            context.window_manager.operator_properties_last("LOAD_DECOMP_OT_operator"),
            "clear_scene",
        )
        layout.label(text="Pickables")
        box_pickable = layout.box()
        box_pickable.prop(
            context.window_manager.operator_properties_last("LOAD_DECOMP_OT_operator"),
            "import_pickables",
        )
        box_pickable.prop(
            context.window_manager.operator_properties_last("LOAD_DECOMP_OT_operator"),
            "pickable_csv",
        )


classes = (
    LOAD_DECOMP_OT_operator,
    LOAD_DECOMP_PT_panel,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)


def unregister():
    for cls in classes:
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()

from habitat_sim import Simulator
from habitat_sim.utils import viz_utils as vut
from habitat_sim.utils.settings import default_sim_settings, make_cfg
from habitat_sim.metadata import MetadataMediator
import magnum as mn

dataset = "data/fp-models/fp-models.scene_dataset_config.json" # modified hssd-hab config to point at fp-models paths
# scene = "grid" # from-scratch scene I made with scale marks
image_size = 1024

sim_settings = default_sim_settings.copy()
# sim_settings["scene"] = scene
sim_settings["width"] = sim_settings["height"] = image_size
sim_settings["enable_hbao"] = True #Ambient Occlusion
sim_settings["default_agent_navmesh"] = False

cfg = make_cfg(sim_settings)
mm = MetadataMediator()
mm.active_dataset = dataset
cfg.metadata_mediator = mm

mm_oam = mm.object_template_manager

###### Dirty test code
glb_path = "data/fp-models/objects/decomposed/00a2b0f3886ccb5ffddac704f8eeec324a5e14c6/00a2b0f3886ccb5ffddac704f8eeec324a5e14c6_dedup.glb"
glb_leaf = "00a2b0f3886ccb5ffddac704f8eeec324a5e14c6_dedup.glb"

json_leaf = "data/fp-models/objects/decomposed/0a3e0a10ec97c41a36939e89b0591d2cfe5e41e1"

print("\n########################################################\n")
my_template = mm_oam.create_new_template(handle=glb_path)
print(f"Render_asset value: {my_template.has_value('render_asset')}")
print(f"file directory: {my_template.file_directory}")
#print(f"Keys in my_template: {my_template.get_keys_and_types()}")
print(f"Render Asset full path location: {my_template.find_value_location('render_asset_fullpath')}")
print(f"Render Asset path: {my_template.render_asset_fullpath}")
#my_template.render_asset = object_leaf
my_template_id = mm_oam.register_template(template=my_template, specified_handle="my_object")
print("\n########################################################\n")    

with Simulator(cfg) as sim:
    # dbv = DebugVisualizer(sim)
    scene_rom = sim.get_rigid_object_manager()
    # scene_oam = sim.get_object_template_manager()

    # load object of interest
    scene_rom.add_object_by_template_handle(object_lib_handle="my_object")

    # render video of a carousel view of the scene
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + 4.0:
        sim.agents[0].scene_node.rotate(
            mn.Rad(mn.math.pi_half / 60.0), mn.Vector3(0, 1, 0)
        )
        sim.step_physics(1.0 / 60.0)
        observations.append(sim.get_sensor_observations())

    # video rendering of carousel view
    video_prefix = f"temp_preview"

    vut.make_video(
        observations,
        "color_sensor",
        "color",
        "output_videos/" + video_prefix,
        open_vid=True,
        video_dims=[image_size, image_size],
    )
from pygltflib import GLTF2
from pathlib import Path as path
import csv

def process_library(library):
    """
    Processes a library of glTF decomp folders.
    """
    results = []
    count = 0
    for folder in library.iterdir():
        if folder.is_dir():
            if args.mode == 0:
                result = process_folder_tri_count(folder)
                results.append(result)
            elif args.mode == 1:
                # Placeholder for mode 1 processing logic
                # This could be implemented to look for instancable objects by triangle count
                # print(f"Mode 1 processing for folder: {folder}")
                # For now, we will just append a placeholder result
                # results.append([folder.stem, 0, 0, 0, 0])
                result = process_folder_duplicate_geo(folder)
                results.append(result)
            count += 1
            if args.debug:  # If debug mode is enabled, limit to first 10 folders
                if count >= 10:
                    break
    return results

def process_folder_tri_count(folder):
    """
    Mode 0:
    Processes a folder containing glTF files and compares original to decomposed triangle counts.
    """
    # Ensure the folder contains a deduplicated glTF file
    uuid = folder.stem
    dedup_file = folder / f"{folder.stem}_dedup.glb"
    if not dedup_file.exists():
        print(f"Skipping {folder} as it does not contain a deduplicated glTF file.")
        return [folder.stem, 0, 0, 0, 0]

    # print(f"UUID is {uuid}")

    # Get original model triangle count
    gltf_model = GLTF2().load(str(dedup_file))
    original_triangle_count = tri_count(gltf_model)

    # count decomp triangles
    parts_triangle_count = 0
    parts_files = folder.glob(f"{uuid}_part_*.glb")
    if not parts_files:
        print(f"No parts files found in {folder}, skipping.")
        return [uuid, original_triangle_count, 0, 0, 0]
    num_parts = len(list(parts_files))
    print(f"{uuid}\n{num_parts} parts found")

    # Process each part file    
    for gltf_file in folder.glob("*_part_*.glb"):
        # print(f"Processing part file: {gltf_file}")
        if not gltf_file.exists():
            print(f"File {gltf_file} does not exist, skipping.")
            continue
        gltf_model = GLTF2().load(str(gltf_file))
        if gltf_model is None:
            print(f"Failed to load {gltf_file}, skipping.")
            continue
        part_triangle_count = tri_count(gltf_model)
        # print(f"Part triangle count for {gltf_file}: {part_triangle_count}")
        parts_triangle_count += part_triangle_count
    # report out
    error = parts_triangle_count - original_triangle_count

    return [uuid, original_triangle_count, parts_triangle_count, error, num_parts]

def process_folder_duplicate_geo(folder):
    """
    Mode 1:
    Processes a folder containing glTF files and flags identical triangle counts.
    """
    uuid = folder.stem
    # print(f"UUID is {uuid}")

    # get part tri counts as dict
    part_tri_counts = {}
    values_to_keys = {}
    parts_files = folder.glob(f"{uuid}_part_*.glb")
    if not parts_files:
        print(f"No parts files found in {folder}, skipping.")
        return [uuid, 0]
    num_parts = len(list(parts_files))
    #print(f"{uuid}\n{num_parts} parts found")

    # Process each part file    
    for gltf_file in folder.glob("*_part_*.glb"):
        # print(f"Processing part file: {gltf_file}")
        if not gltf_file.exists():
            print(f"File {gltf_file} does not exist, skipping.")
            continue
        part_number = str(gltf_file.stem.split('_')[-1])  # Extract part number from filename
        gltf_model = GLTF2().load(str(gltf_file))
        if gltf_model is None:
            print(f"Failed to load {gltf_file}, skipping.")
            continue
        part_triangle_count = tri_count(gltf_model)
        part_tri_counts[part_number] = part_triangle_count
    
    if args.verbose:
        print(f"Part triangle counts for {uuid}:\n{part_tri_counts}")
        
    # find possible duplicates
    for key, value in part_tri_counts.items():
        if value not in values_to_keys:
            values_to_keys[value] = [key]
        else:
            values_to_keys[value].append(key)
    if args.verbose:
        print(f"Possible duplicates:\n{values_to_keys}")

    identical_value_pairs = []
    for value_list in values_to_keys.values():
        if len(value_list) > 1:
            identical_value_pairs.append(value_list)
    # Prepare the output
    if args.verbose:
        print(f"Identical value pairs for {uuid}:\n{identical_value_pairs}")
    identical_value_pairs.insert(0, uuid)

    return identical_value_pairs

def tri_count(gltf):
    """
    Calculates the total number of triangles in a glTF model.
    """
    total_triangles = 0
    for mesh in gltf.meshes:
        for primitive in mesh.primitives:
            # Ensure the primitive's mode is TRIANGLES (4)
            # Other modes like POINTS (0), LINES (1), LINE_LOOP (2), etc., exist
            if primitive.mode is None or primitive.mode == 4:  # Default mode is TRIANGLES if not specified
                if primitive.indices is not None:
                    accessor = gltf.accessors[primitive.indices]
                    # For triangles, each set of 3 indices forms one triangle
                    total_triangles += accessor.count // 3
                else:
                    # If no indices are provided, assume non-indexed triangles
                    # The number of vertices must be a multiple of 3
                    position_accessor_index = primitive.attributes.POSITION
                    position_accessor = gltf.accessors[position_accessor_index]
                    total_triangles += position_accessor.count // 3
    return total_triangles

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Process a folder of glTF files, comparing original with decomposed elements.")
    parser.add_argument("folder", type=str, help="Path to the folder containing glTF files.")
    parser.add_argument("-m", "--mode", type=int, choices=[0, 1], help="Operation mode.\n0:Compare original and decomposed triangle counts\n1: Look for instancable object by triangle count.")
    parser.add_argument("-o", "--output", type=str, default="decomp_check", help="Output CSV file name.")
    parser.add_argument("-d", "--debug", action="store_true", help="run only the first ten folders for debugging purposes.")
    parser.add_argument("-v", "--verbose", action="store_true", help="Enable verbose output for debugging.")
    args = parser.parse_args()

    library = path.resolve(path(args.folder))
    if not library.exists():
        raise ValueError(f"{args.folder} does not exist.")
    if not library.is_dir():
        raise ValueError(f"{args.folder} is not a valid directory.")
    if args.debug:
        print("Running in debug mode, limiting to first 10 folders.")
    
    output_file = library / f"{args.output}_{args.mode}.csv"
    if args.mode == 0:
        print("Running in triangle count audit mode.")
        header = ["UUID", "Original Triangle Count", "Decomposed Triangle Count", "Error", "Number of Parts"]
    elif args.mode == 1:
        print("Running in instancable object detection mode.")
        header = ["UUID", "Set_1", "Set_2", "Set_3", "Set_4", "Set_5", "Set_6", "Set_7", "Set_8", "Set_9", "Set_10"]

    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)
        csv_data = process_library(library)
        writer.writerows(csv_data)
# Check gltf texture sizes

from pygltflib import GLTF2
from PIL import Image
import io
import os
import numpy as np
import glob

def get_texture_info(gltf_path: str) -> list:
    textures = []
    gltf = GLTF2.load(gltf_path)
    base_name = os.path.basename(gltf_path)
    blob = gltf.binary_blob()
    for idx, image in enumerate(gltf.images):
        texture_info = {}
        texture_info['glb'] = base_name
        texture_info['idx'] = idx
        if image.bufferView is not None:
       # If image data is stored in a buffer view, retrieve it
            buffer_view_idx = image.bufferView
            buffer_view = gltf.bufferViews[buffer_view_idx]
            start = buffer_view.byteOffset
            end = start + buffer_view.byteLength
            buffer_idx = buffer_view.buffer
            buffer = gltf.buffers[buffer_idx]

            #print(buffer.__dict__.keys())
            image_data = blob[start:end]
            # Now you have the raw image data (e.g., JPEG or PNG bytes) in `image_data`

            # Use Pillow to open the image data
            try:
                img = Image.open(io.BytesIO(image_data))
                width, height = img.size
                texture_info["width"] = width
                texture_info["height"] = height
                #img.show()
            except Exception as e:
                print(f"Error loading image: {e}")
            
        elif image.uri is not None:
            texture_info["type"] = "uri"
            texture_info["width"] = 0
            texture_info["height"] = 0
            # Handle base64 encoded data URI images (if applicable)
            # This would involve decoding the URI and then using Pillow
            print("Handling of data URI not shown in this example.")
        textures.append(texture_info)
    return textures

glb_files = glob.glob('/home/jseagull/dev/fp-models/objects/decomposed/**/*_part_*.glb')
for file in glb_files:
    tex_info = get_texture_info(file)
    for big_tex in (tex for tex in tex_info if tex['width']>1024):
        print(f"{big_tex['glb']}: Texture {big_tex['idx']} is {big_tex['width']}")

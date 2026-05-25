import os
import importlib

Modelname = 'TD3model'
tfmodel = 'td3_model'


############ Add lines to model.py and __init__.py to set TF_USE_LEGACY_KERAS ############

## Model.py
file_path = os.path.join(os.path.dirname(__file__), Modelname, 'model.py')
# Lines to add
new_lines = "import os\nos.environ[\"TF_USE_LEGACY_KERAS\"] = \"1\"\n\n"

# Read the current content
with open(file_path, 'r') as f:
    content = f.read()

# Check if lines already exist
if 'TF_USE_LEGACY_KERAS' not in content:
    # Write the new lines + original content
    with open(file_path, 'w') as f:
        f.write(new_lines + content)
    print(f"Added lines to {file_path}")
else:
    print("Lines already exist in the file")

## __init__.py
file_path = os.path.join(os.path.dirname(__file__), Modelname, '__init__.py')
# Lines to add
new_lines = "import os\nos.environ[\"TF_USE_LEGACY_KERAS\"] = \"1\"\n\n"

# Read the current content
with open(file_path, 'r') as f:
    content = f.read()

# Check if lines already exist
if 'TF_USE_LEGACY_KERAS' not in content:
    # Write the new lines + original content
    with open(file_path, 'w') as f:
        f.write(new_lines + content)
    print(f"Added lines to {file_path}")
else:
    print("Lines already exist in the file")


######## Load Python package and create TensorFlow model #####################
# import TD3model
TD3model = importlib.import_module(Modelname)
model = TD3model.load_model(load_weights=True)
model.summary()

# Save as SavedModel
save_path = os.path.join(os.path.dirname(__file__), tfmodel)
model.save(save_path)


############ load tensorflow model and convert to TFLite ############
os.environ["TF_USE_LEGACY_KERAS"] = "1"

import tensorflow as tf
import numpy as np

# Convert to TFLite
converter = tf.lite.TFLiteConverter.from_saved_model(tfmodel)
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS]
converter.optimizations = [tf.lite.Optimize.DEFAULT]  # Quantization (optional)

converter.experimental_enable_resource_variables = False
converter.allow_custom_ops = False

tflite_model = converter.convert()

# Save TFLite model
with open(tfmodel + '.tflite', 'wb') as f:
    f.write(tflite_model)


########## Read the TFlite model and convert to C array ############
# Read the TFLite model
with open(tfmodel + '.tflite', 'rb') as f:
    tflite_model = f.read()

# Convert to C array - split on hex value boundaries
hex_list = [f'0x{b:02x}' for b in tflite_model]
lines = []
current_line = []

for hex_val in hex_list:
    current_line.append(hex_val)
    if len(current_line) == 12:  # 12 values per line for readability
        lines.append(', '.join(current_line))
        current_line = []

if current_line:  # Add remaining values
    lines.append(', '.join(current_line))
c_code = "#ifndef td3_MODEL_H\n"
c_code += "#define td3_MODEL_H\n\n"
c_code += 'const unsigned char td3_model[] = {\n'
c_code += ',\n'.join(lines)
c_code += '\n};\nconst int td3_model_len = ' + str(len(tflite_model)) + ';'
c_code += "\n\n#endif // td3_MODEL_H"

with open('td3_model.h', 'w') as f:
    f.write(c_code)

print("done")
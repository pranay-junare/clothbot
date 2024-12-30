# python3 ./eval_create.py  file1.hdf5 file2.hdf5.py
# python3 ./eval_create.py  flingbot-shirt-eval.hdf5 flingbot-shirt-eval-10.hdf5
# Script to reduce the number of samples present in the hdf5 file and store it in new file.

from h5py import File as HDF5
from filelock import FileLock
import h5py
import sys

if __name__ == "__main__":
    input_path = sys.argv[1]
    output_path = sys.argv[2]

    with FileLock(input_path + '.lock'):
        with HDF5(input_path, 'r') as input_file, HDF5(output_path, 'w') as output_file:
            # Get all keys and take the first 10
            keys = list(input_file.keys())[:10]
            print(f"Copying the first {len(keys)} keys: {keys}")

            # Copy each dataset or group to the new file
            for key in keys:
                obj = input_file[key]
                if isinstance(obj, h5py.Group):
                    # Copy group recursively
                    def copy_group(source_group, dest_group):
                        for sub_key in source_group:
                            sub_obj = source_group[sub_key]
                            if isinstance(sub_obj, h5py.Dataset):
                                dest_group.create_dataset(sub_key, data=sub_obj[...])
                            elif isinstance(sub_obj, h5py.Group):
                                new_group = dest_group.create_group(sub_key)
                                copy_group(sub_obj, new_group)
                    
                    new_group = output_file.create_group(key)
                    copy_group(obj, new_group)
                elif isinstance(obj, h5py.Dataset):
                    output_file.create_dataset(key, data=obj[...])
                else:
                    print(f"Skipping unknown object type for key: {key}")

    print(f"New HDF5 file with first 10 keys saved to: {output_path}")

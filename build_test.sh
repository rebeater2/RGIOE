output_path="cmake-build-test"
cmake -B ${output_path}  -GNinja -DEST_ACCE_SCALE=ON -DEST_GYRO_SCALE=ON -DRECORDER=ON -DEST_GNSS_LEVEL_ARM=ON -DEST_ODO_SCALE=ON
ninja -C ${output_path}

cmake -B ${output_path}  -GNinja -DEST_ACCE_SCALE=OFF -DEST_GYRO_SCALE=OFF -DRECORDER=ON -DEST_GNSS_LEVEL_ARM=OFF -DEST_ODO_SCALE=OFF
ninja -C ${output_path}

cmake -B ${output_path} -GNinja -DEST_ACCE_SCALE=ON -DEST_GYRO_SCALE=ON -DRECORDER=ON -DEST_GNSS_LEVEL_ARM=OFF -DEST_ODO_SCALE=OFF
ninja -C ${output_path}

cmake -B ${output_path}  -GNinja -DEST_ACCE_SCALE=OFF -DEST_GYRO_SCALE=OFF -DRECORDER=OFF -DEST_GNSS_LEVEL_ARM=OFF -DEST_ODO_SCALE=OFF
ninja -C ${output_path}
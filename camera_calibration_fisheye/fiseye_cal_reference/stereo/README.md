## OpenCV C++ Stereo Fisheye Calibration

This contains a source file to calibrate a stereo system comprising of fisheye lenses. It calibrates the extrinsics and the intrinsics of the cameras without any initial guesses. If you are looking for stereo calibration with lenses which follow the pinhole model check [here](https://github.com/sourishg/stereo_calibration).

### Dependencies

- OpenCV
- popt

### Compilation

Compile all the files using the following commands.

```bash
mkdir build && cd build
cmake ..
make
```

Make sure your are in the `build` folder to run the executables.

### Data

Some sample calibration images are stored in the `imgs` folder.

### Running calibration

Run the executable with the following command

```bash
./calibrate -w [board_width] -h [board_height] -s [square_size] -n [num_imgs] -d [img_dir] -l [left_img_prefix] -r [right_img_prefix] -o [calib_file]
```

For example if you use the images in the `imgs` folder run the following command

```bash
./calibrate -w 9 -h 6 -s 0.02423 -n 29 -d ../imgs/ -l left -r right -o cam_stereo.yml
```
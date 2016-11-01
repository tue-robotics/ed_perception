ED Perception [![Build Status](https://travis-ci.org/tue-robotics/ed_perception.svg?branch=master)](https://travis-ci.org/tue-robotics/ed_perception)
======

Plugins for classifying entities based on their attached RGBD measurements.

## Installation

Depends on:
- https://github.com/tue-robotics/ed.git

Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src
    git clone https://github.com/tue-robotics/ed_perception.git

And compile

    cd <your_catkin_workspace>:
    catkin_make
    
## Tutorial

All ED tutorials can be found in the ed_tutorials package: https://github.com/tue-robotics/ed_tutorials

### Capturing images

Start the robot

    astart
    amiddle

Position the robot in front of the objects, and let the head look at the objects. Open a robot console (e.g. for AMIGO):

    robot-console
    inspect hallway_table

or

    amigo-console

Now you can capture an image and save it to disk using:

    amigo.ed.save_image(path="/some/path")

This will store the RGBD-image (color + depth) and meta-data (such as the timestamp and 6D pose) to the path you specified. If this path does not yet exist, it will be created. The filename of the file will be the date and timestamp of the image captured.

### Annotating the images

**Before you can annotate the images, a 3D (ED) model of the supporting furniture (e.g. the table on which the objects are positioned)**. Let's say the name of this model is `my-lab/table`.

**To streamline the annotation, it is also possible to load the available object types in the GUI and auto-annotate the supporting objects.** For this a number of conditions need to be met:
  * The images must be stored in a folder structure so that the GUI can derive the supporting object from the parent directory, where `on_top_of` is an area defined in the `table` model:

like this:
  
    images
    |------ on_top_of_table
    |       |   img1.rgbd
    |       |   img1.png
    |       |   ...
    |
    |------ shelf3_cabinet
    |       |   img2.rgbd
    |       |   img2.png
    |       |   ...
    ...

  * The environment variable `ROBOT_ENV` must be set to a world name defined in `ed_object_models` (`my-lab`) and in `robocup_knowledge`. 
  * The world description must contain a model (`my-lab/table`) in a composition defined in `my-lab/model.yaml` (see the [ED tutorials](https://github.com/tue-robotics/ed_tutorials), specifically tutorials 1-5). This description is automatically loaded when the annotation GUI starts.
  * `robocup_knowledge/src/robocup_knowledge/my-lab` must contain a python database of the objects that you want to annotate (typically the objects that the robot needs to be able to recognize in `my-lab`).
  * To load the object labels, run a `roscore`, and run 

    rosrun ed_perception load_object_types

Now you can start the annotator with the path in which you stored the images:

    rosrun ed_perception annotation-gui /some/path

and cycle through the images using the arrow keys. 

If you don't have auto-annotation of the supporting object, the segmentation is probably pretty bad. This will however change once you add the supporting furniture.

  * In the annotation-gui, type the name of the supporting entity, in our case `my-lab/table`. You should see the entity name appearing.
  * Once you typed the name, press enter. The color of the name should change to red. This means that it is selected
  * Left-click in the image on the location where the supporting (the table) is. A label should appear
  * If you made a mistake, right-click on the blue sphere to delete it.

Go through all the images and annotate the supporting entity.

By default, the area that is used for segmentation is `on_top_of`. You can however specify another area. If you want this:

  * Type `area:<name-of-area>`, for example `area:shelf1` and press enter
  * click anywhere on the image. You should see the area name change in the upper status bar

Once an image has the supporting entity annotated, segmentation should improve. Now you can annotate the rest objects in the images, using the same process:

  * Type the name of the object an press enter
    * **If you loaded the object types, you can use auto-completion! Use the up and down arrows to select the object**
  * Left-click on the (middle of the) object in the image
  * If segment rectangles overlap, don't annotate in the overlapping region, because it is undefined which of the two rectangles will then be annotated.
  * *Tip: type the name of the object, then go through all the images and click on it. In general this is faster then re-typing the name of the object, even with auto-completion*

If you encounter any images that are useless for annotation/training, it is possible to exclude them. If you do that, the image crawler will skip the image in the future, in the GUI as well as while training or testing. Excluding an image is done by typing exclude, pressing enter and clicking in the image. This can only be undone by manually opening the json file with the metadata of the excluded image and setting exclude to false.

You can always exit the annotation-gui by pressing ESC. Your progress will be saved (in the json-meta-data files).

### Segmenting the images for training of the deep learning perception

Once you have a decent number of annotations of all the objects you want your robot to recognize, you might want to train a neural network using the segments containing the annotated objects. Good news! You can do that! By running

    rosrun ed_perception store_segments /path/containing/annotated/images/ /target_directory/

the annotated segments are cut out and stored in the following directory structure:

    target_directory
    |------ coke
    |       |   img1.png
    |       |   img2.png
    |       |   ...
    |
    |------ toilet_paper
    |       |   img1.png
    |       |   img11.png
    |       |   ...
    ...

### TODO: Training the awesome deep learning peception module

### Training the OLD perception modules (deprecated)

Go to the package in which the perception modules are stored:

    roscd ed_perception_models
    cd models

Create a new folder with the name of the current environment:

   mkdir $ROBOT_ENV

Within this directory, create a file called `parameters.yaml` with a content like this:

    modules:
    - lib: libcolor_matcher.so
      classification:
        color_margin: 0.02
    - lib: libsize_matcher.so
      classification:
        size_margin: 0.01

This determines which perception modules are used, with which parameters. Now you can train the models for these perception modules, using the `train-perception` tool. This takes two arguments:

    rosrun ed_perception train-perception <config-file> <image-file-or-directory>

So in our case:

    rosrun ed_perception train-perception `rospack find ed_perception_models`/models/$ROBOT_ENV/parameters.yaml /path/to/images

### Testing the OLD perception models (deprecated)

    rosrun ed_perception test-perception `rospack find ed_perception_models`/models/$ROBOT_ENV/parameters.yaml /path/to/images

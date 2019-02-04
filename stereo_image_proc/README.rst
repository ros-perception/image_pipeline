=================
stereo_image_proc
=================

Original Authors:

* Patrick Mihelich
* Kurt Konolige
* Jeremy Leibs

Original Maintainer:

* Vincent Rabaud <vincent.rabaud AT gmail DOT com>

-----------------------
VisionWorks Integration
-----------------------

Author:

* Zach LaCelle <zlacelle@mitre.org>

NVIDIA VisionWorks is a set of libraries and APIs enabing
CUDA-accelerated parallel computing of vision processing
algorithms. It provides an easy-to-use, optimized method
of adding parallel compute capabilities to a vision processing
project.

Documentation of VisionWorks is here (account creation required):

https://developer.nvidia.com/embedded/visionworks

VisionWorks makes use of the OpenVX standard, maintained by
the Khronos group. Details of the OpenVX specification are
available here:

https://www.khronos.org/openvx/

VisionWorks is supported on the NVIDIA Drive and NVIDIA TX2
systems, as well as on traditional graphics cards--making it
useful for integration with autonomy applications.

Tested Versions
...............

This software has been tested with VisionWorks 1.6, which includes
CUDA 9.0.

Installation
............

To enable VisionWorks support, download VisionWorks via JetPack and
install locally. This will also install CUDA.

After installation of VisionWorks and CUDA, the VisionWorks interface
libraries must be custom-compiled.  To link into ROS nodelet libraries,
these static libs must be compiled with Position-Independent Code (-fPIC).

Manually edit the makefile at /usr/share/visionworks/sources, and add:

``export CXXFLAGS+=-fPIC``

If you don't do this, the nodelet will fail at runtime.

To make Package-Config work with VisionWorks, you must also fix the
visionworks.pc file located at /usr/lib/pkgconfig/visionworks.pc, replacing
the corresponding lines with:

::
   
   prefix=/usr/share/visionworks/sources
   exec_prefix=${prefix}/bin/<arch>/linux/release
   libdir=${prefix}/libs/<arch>/linux/release
   includedir=${prefix}/nvxio/include

Note that you need to replace <arch> with your system's architecture, shown
with the output of ``uname -m``

At this point, you should be ready to compile stereo_image_proc with
VisionWorks.

Compiling
.........

The CMakeLists.txt for stereo_image_proc searches for cudart-X.X and
cuda-X.X with pkg-config (where X.X is the supported version number)
and, if found, searches for visionworks with pkg-config. Assuming these
are found, it then searches for libnvx and libovx, and generates a
libstereo_image_proc_visionworks.so library linked against these. This
library is then linked into libstereo_image_proc.so. It also passes
-DVISIONWORKS_ACCELERATION to the compiler, which is used in
precompiler directives to enable VisionWorks blocks of code.

If these packages and libraries are not found, it continues compiling
without support for VisionWorks.

Generating Documentation
------------------------

VisionWorks integration is documented through Doxygen comments.

To generate Doxygen documentation, run the following command from the
stereo_image_proc directory:

::

   rosdoc_lite .

The output will be in doc/html/c++. To open it, run:

::
   
   firefox doc/html/c++/index.html

For more information, see the rosdoc_lite documentation here:
   
http://wiki.ros.org/rosdoc_lite      
   
Enabling VisionWorks
--------------------

VisionWorks is enable-able at runtime using ROS dynamic-reconfigure. The
corresponding flag is "use_visionworks." Set the flag to True or check
the box to switch to running VisionWorks-enabled code. Note that you
can switch at runtime without restarting.

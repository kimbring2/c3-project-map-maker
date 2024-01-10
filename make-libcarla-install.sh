# makes libcarla-install dir for Udacity project
LIBCARLA_INSTALL_DIRECTORY=libcarla-install
CARLA_SOURCE_DIRECTORY=/media/kimbring2/be356a87-def6-4be8-bad2-077951f0f3da/carla

if [ -d "$LIBCARLA_INSTALL_DIRECTORY" ]; then
  echo "$LIBCARLA_INSTALL_DIRECTORY already exists.  Not doing anything."
fi

if [ ! -d "$LIBCARLA_INSTALL_DIRECTORY" ]; then
  echo "$LIBCARLA_INSTALL_DIRECTORY does not exist.  Creating."
  mkdir $LIBCARLA_INSTALL_DIRECTORY
  cd $LIBCARLA_INSTALL_DIRECTORY
  mkdir include
  cd include
  ln -s $CARLA_SOURCE_DIRECTORY/LibCarla/source/carla .
  mkdir system
  cd system
  ln -s $CARLA_SOURCE_DIRECTORY/Build/boost-*-install/include/boost .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/libpng-*-install/include/* .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/recast-*-install/include/recast .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/rpclib-*-libstdcxx-install/include/rpc .
  cd ..
  cd ..
  mkdir lib
  cd lib
  ln -s $CARLA_SOURCE_DIRECTORY/Build/libcarla-client-build.release/LibCarla/cmake/client/*.a .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/boost-*-install/lib/*.a .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/boost-*-install/lib/*.so* .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/libpng-*-install/lib/*.a .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/libpng-*-install/lib/*.so* .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/recast-*-install/lib/*.a .
  ln -s $CARLA_SOURCE_DIRECTORY/Build/rpclib-*-libstdcxx-install/lib/*.a .
  cd ..
  cd ..
fi

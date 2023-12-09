# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/yugen/esp/esp-idf/components/bootloader/subproject"
  "/Users/yugen/Universidad/Cuarto/SBC/Proyecto_Final/Pet-Feeder/build/bootloader"
  "/Users/yugen/Universidad/Cuarto/SBC/Proyecto_Final/Pet-Feeder/build/bootloader-prefix"
  "/Users/yugen/Universidad/Cuarto/SBC/Proyecto_Final/Pet-Feeder/build/bootloader-prefix/tmp"
  "/Users/yugen/Universidad/Cuarto/SBC/Proyecto_Final/Pet-Feeder/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/yugen/Universidad/Cuarto/SBC/Proyecto_Final/Pet-Feeder/build/bootloader-prefix/src"
  "/Users/yugen/Universidad/Cuarto/SBC/Proyecto_Final/Pet-Feeder/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/yugen/Universidad/Cuarto/SBC/Proyecto_Final/Pet-Feeder/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/yugen/Universidad/Cuarto/SBC/Proyecto_Final/Pet-Feeder/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()

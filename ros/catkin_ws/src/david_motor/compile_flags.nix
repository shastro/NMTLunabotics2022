# This script generates compile_flags suitable for NixOS
# installations; it can be used instead of the normal
# `compile_flags.txt` file included in the Git on such systems.

{ pkgs ? import <nixpkgs> {} }:

with pkgs;
writeTextFile {
  name = "compile_flags.txt";
  text = ''
    -DROSCONSOLE_BACKEND_LOG4CXX
    -DROS_BUILD_SHARED_LIBS=1
    -DROS_PACKAGE_NAME="david_motor"
    -I../../devel/include
    -I../../../../teknic/inc/inc-pub
    -Iinclude
    -I../../../sys-include
    -I../../../sys-include/xmlrpcpp
    -I${boost.dev}/include
    -I${log4cxx}/include
  '';
}

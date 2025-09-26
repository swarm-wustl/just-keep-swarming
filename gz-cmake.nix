{ stdenv, fetchFromGitHub, cmake, pkg-config }:

stdenv.mkDerivation rec {
  pname = "gz-cmake";
  version = "5_5.0.0";

  src = fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-cmake";
    rev = "gz-cmake${version}";
    sha256 = "";
  };

  nativeBuildInputs = [ cmake pkg-config ];

  meta = {
    description = "CMake modules for Gazebo (gz-cmake)";
    license = stdenv.lib.licenses.asl20;
    platforms = stdenv.lib.platforms.linux;
  };
}

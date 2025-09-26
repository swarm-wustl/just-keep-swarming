{ lib, stdenv, fetchFromGitHub, cmake, ogre, bullet, eigen, pkgconfig }:

stdenv.mkDerivation rec {
  pname = "ign-gazebo";
  version = "9.4.0";

  src = fetchFromGitHub {
    owner = "gazebosim";
    repo = "gz-sim";
    rev = "gz-sim9_9.4.0";
    sha256 = "";
  };

  nativeBuildInputs = [ cmake pkgconfig ];

  buildInputs = [
    ogre
    bullet
    eigen
  ];

  cmakeFlags = [
    "-DBUILD_GUI=ON"
    "-DBUILD_TESTING=OFF"
    # "-DCMAKE_INSTALL_PREFIX=/nix/store/...-ign-gazebo-${version}" # often default is fine
  ];

  # maybe patches if needed

  installPhase = ''
    mkdir -p $out/bin
    # actual install
    ${lib.optionalString stdenv.isDarwin "# some adjustments"} 
    make install
  '';

  meta = with lib; {
    description = "Ignition Gazebo sim – robotics simulator (Fortress)";
    license = licenses.bsd3; 
    maintainers = with maintainers; [ /* your handle */ ];
  };
}

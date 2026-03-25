{ pkgs
, gzCmake
, pname ? "ign-gazebo-src"
, version ? "unstable"
, rev ? "main"
, sha256 ? null
, extraBuildInputs ? []
, extraNativeBuildInputs ? []
# , cmakeFlags ? 
}:

let
  lib = pkgs.lib;
  # If caller didn't provide a sha256, use a fake placeholder so `nix` prints
  # the real one on first attempt. Caller should replace it for deterministic builds.
  sha = if sha256 == null then lib.fakeSha256 else sha256;
in

pkgs.stdenv.mkDerivation rec {
  name = "${pname}-${version}";
  inherit version;

  src = pkgs.fetchFromGitHub {
    owner = "ignitionrobotics";
    repo  = "ign-gazebo";
    rev   = rev;
    sha256 = sha;
  };

  # Tools required for configure/build
  nativeBuildInputs = (with pkgs; [
    cmake
    gzCmake
    pkg-config
    git
    ninja
    gnumake
  ]) ++ extraNativeBuildInputs;

  # Common libraries the project expects. Caller may extend/override.
  buildInputs = (with pkgs; [
    boost
    eigen
    protobuf
    yaml-cpp
    tinyxml2
    bullet
    assimp
    ogre
    openssl
    opencv
    pkg-config
  ]) ++ extraBuildInputs;

  # Ensure install path is $out
  cmakeFlags = [
    "-DCMAKE_BUILD_TYPE=Release"
    "-DBUILD_TESTING=OFF"
    "-DCMAKE_INSTALL_PREFIX=$out"
    "-DCMAKE_MODULE_PATH=${gzCmake}/share/cmake/gz-cmake4"
  ];

  # Configure / build / install phases (straightforward translation of upstream)
  configurePhase = ''
    mkdir -p build
    cd build
    cmake .. ${lib.concatStringsSep " " cmakeFlags}
  '';

  buildPhase = ''
    cd build
    # prefer parallel build if nproc is available
    make -j$(command -v nproc >/dev/null 2>&1 && nproc || echo 1)
  '';

  installPhase = ''
    cd build
    make install
  '';

  # Small hygiene: strip debug symbols if desired (leave to caller if not wanted)
  dontStrip = false;

  meta = with lib; {
    description = "Ignition Gazebo - build from source (derivation)";
    homepage = "https://github.com/ignitionrobotics/ign-gazebo";
    license = licenses.asl20;
    maintainers = [];
  };
}

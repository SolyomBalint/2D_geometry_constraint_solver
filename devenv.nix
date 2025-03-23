{
  pkgs,
  lib,
  config,
  inputs,
  ...
}:
let
  pkgs_unstable = import inputs.unstable_nixkpgs { system = pkgs.stdenv.system; };
in
# pkgs_stable = import inputs.stable_latest_nixpkgs { system = pkgs.stdenv.system; };
# nixos_2025_01_22 = import inputs.nixos_2025_01_22 { system = pkgs.stdenv.system; };
# nixpkgs_2025_01_22 = import inputs.nixpkgs_2025_01_22 { system = pkgs.stdenv.system; };
{
  cachix.enable = true;
  cachix.pull = [ "pre-commit-hooks" ];
  # Environmental variables
  env.GREET = "Welcome to 2D geometry constraint solver development environment!";

  # packages
  packages = with pkgs_unstable; [
    ## Build tools
    gcc
    gnumake
    ninja
    # This is built from another commit because the one on nixpkgs is faulty for flakes
    # Prefer nixpkgs to keep it usable on other platforms as well
    cmake

    ## Tools
    doxygen
    valgrind
    virtualglLib

    ## Build deps
    imgui
    glfw
    pkg-config
    imnodes
    spdlog
    argparse
    boost.dev
    boost.out
  ];

  # languages.python = {
  #   enable = true;
  #   version = "3.13";
  #   venv = {
  #     enable = true;
  #     requirements = '''';
  #   };
  # };

  # Scripts
  enterShell = ''
    echo $GREET
  '';
}

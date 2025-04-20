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
{
  cachix.enable = true;
  cachix.pull = [ "pre-commit-hooks" ];
  # Environmental variables
  env.GREET = "Welcome to 2D geometry constraint solver development environment!";
  # env.DEVENV_GCC = "${pkgs_unstable.libgcc.out}/bin/gcc";
  # env.DEVENV_GPP = "${pkgs_unstable.libgcc.out}/bin/g++";

  # packages
  packages = with pkgs_unstable; [
    ## Build tools
    libgcc
    gcc14
    gnumake
    cmake
    ninja
    pkg-config

    ## Tools
    doxygen
    valgrind
    virtualglLib
    sass

    ## Build deps
    spdlog # MIT licence
    argparse # MIT licence
    boost.dev # Boost software licence
    boost.out

    ## Example gui dependencies
    (python313.withPackages (
      ps: with ps; [
        graph-tool
      ]
    ))
    ### For graph_tool
    gtk3
    librsvg
    glib
    gobject-introspection
    cairo
  ];

  languages.python = {
    enable = true;
    version = "3.13";
    venv = {
      enable = true;
      requirements = ''
        nanobind

        scipy
        PyGObject
        numpy
        pycairo
        matplotlib
      '';
    };
  };
  enterShell = ''
    echo $GREET
  '';
}

{
  pkgs,
  lib,
  config,
  inputs,
  ...
}:
let
  pkgs_unstable = import inputs.unstable_nixkpgs { system = pkgs.stdenv.system; };
  gcc15Stdenv = pkgs_unstable.gcc15Stdenv;
in
{
  cachix.enable = true;
  cachix.pull = [ "pre-commit-hooks" ];

  # Environmental variables
  env.GREET = "Welcome to 2D geometry constraint solver development environment!";
  env.CONAN_HOME = "${config.devenv.root}/.conan2";

  # packages
  packages = with pkgs_unstable; [
    ## Build tools
    gcc15Stdenv.cc
    gnumake
    ninja
    cmake
    clang-tools
    clang
    pkg-config

    ## Tools
    doxygen
    gdb
    valgrind
    virtualglLib
    sass

    ## Example gui dependencies
    (python313.withPackages (ps: with ps; [ graph-tool ]))
    ### For graph_tool
    gtk3
    gsettings-desktop-schemas
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
        conan
        conan-stubs
        mypy

        scipy
        PyGObject
        numpy
        pycairo
        matplotlib
      '';
    };
  };
  enterShell = ''
    export CC=clang
    export CXX=clang++

    echo ""
    echo "██████╗ ███╗   ███╗███████╗    ██╗██╗████████╗"
    echo "██╔══██╗████╗ ████║██╔════╝    ██║██║╚══██╔══╝"
    echo "██████╔╝██╔████╔██║█████╗      ██║██║   ██║   "
    echo "██╔══██╗██║╚██╔╝██║██╔══╝      ██║██║   ██║   "
    echo "██████╔╝██║ ╚═╝ ██║███████╗    ██║██║   ██║   "
    echo "╚═════╝ ╚═╝     ╚═╝╚══════╝    ╚═╝╚═╝   ╚═╝   "
    echo ""

    # Check git commit template
    echo "Checking whether git commit template is set for local project"
    if git config --get commit.template >/dev/null 2>&1; then
        echo "Commit template is already set, continuing"
    else
        TEMPLATE_CMD="git config commit.template .git_commit_msg_template"
        echo "No commit template configured, setting it for local project"
        echo "Executing: $TEMPLATE_CMD"
        $TEMPLATE_CMD
    fi
    echo

    # Check conan profile
    echo "Checking whether conan2 profile exists"
    if [ -d "$CONAN_HOME" ] && [ -n "$(ls -A "$CONAN_HOME" 2>/dev/null)" ]; then
        echo "Conan profile directory exists and contains files"
    else
        CONAN_CMD="conan profile detect --force"
        echo "Directory doesn't exist or is empty, creating conan profile"
        echo "Executing: $CONAN_CMD"
        $CONAN_CMD
    fi
    echo

    export XDG_DATA_DIRS="${pkgs_unstable.gsettings-desktop-schemas}/share/gsettings-schemas/${pkgs_unstable.gsettings-desktop-schemas.name}:${pkgs_unstable.gtk3}/share/gsettings-schemas/${pkgs_unstable.gtk3.name}:$XDG_DATA_DIRS"
    export GI_TYPELIB_PATH="${pkgs_unstable.gtk3}/lib/girepository-1.0:${pkgs_unstable.gobject-introspection}/lib/girepository-1.0:$GI_TYPELIB_PATH"
  '';
}

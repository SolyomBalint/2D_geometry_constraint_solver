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
  env.CONAN_HOME = "${config.devenv.root}/.conan2";

  # env.DEVENV_GCC = "${pkgs_unstable.libgcc.out}/bin/gcc";
  # env.DEVENV_GPP = "${pkgs_unstable.libgcc.out}/bin/g++";

  scripts.build_linux_gcc_debug.exec = ''
    echo "If the script fails on nixos run it in an fhs shell"
    conan install $DEVENV_ROOT --remote=conancenter --build=missing \
    -pr $DEVENV_ROOT/conan_profiles/Linux/LinuxGccStd20DebugProfile

    cmake -B $DEVENV_ROOT/build/Debug -S $DEVENV_ROOT \
    -DCMAKE_TOOLCHAIN_FILE=$DEVENV_ROOT/build/Debug/generators/conan_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Debug

    # This is a horrible hack, for the lack of correct clangd configuration
    cp $DEVENV_ROOT/build/Debug/compile_commands.json $DEVENV_ROOT/build

    cmake --build $DEVENV_ROOT/build/Debug
  '';

  scripts.build_linux_gcc_release.exec = ''
    echo "If the script fails on nixos run it in an fhs shell"
    conan install $DEVENV_ROOT --remote=conancenter --build=missing \
    -pr $DEVENV_ROOT/conan_profiles/Linux/LinuxGccStd20ReleaseProfile

    cmake -B $DEVENV_ROOT/build/Release -S $DEVENV_ROOT \
    -DCMAKE_TOOLCHAIN_FILE=$DEVENV_ROOT/build/Release/generators/conan_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release

    # This is a horrible hack, for the lack of correct clangd configuration
    cp $DEVENV_ROOT/build/Release/compile_commands.json $DEVENV_ROOT/build

    cmake --build $DEVENV_ROOT/build/Release
  '';

  # packages
  packages = with pkgs_unstable; [
    ## Build tools
    libgcc
    gcc15
    gnumake
    cmake
    llvmPackages_20.clang-tools
    llvmPackages_20.clangNoCompilerRtWithLibc # in the local environemnt the goal is to be able to use both compilers
    # for checking for mistakes, so we leave out the LLVM cpp libs to avoid linking issues
    gdb
    ninja
    pkg-config

    ## Tools
    doxygen
    valgrind
    virtualglLib
    sass

    ## Build deps
    # spdlog # MIT licence
    # argparse # MIT licence
    # stduuid

    ## Example gui dependencies
    (python313.withPackages (ps: with ps; [ graph-tool ]))
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
  '';
}

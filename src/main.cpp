#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imnodes.h"

// General STD/STL headers
#include <format>

// Custom headers
#include <rendering/graph_render.hpp>

// Thirdparty headers not needed for rendering
#include <argparse/argparse.hpp>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#define GL_SILENCE_DEPRECATION
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

constexpr const int WINDOW_HEIGHT = 500;
constexpr const int WINDOW_WIDTH = 500;

auto mainLogger = spdlog::stdout_color_mt("MAIN");

int ConvertLogLevel(const std::string& input)
{
    if (input == "TRACE") {
        return spdlog::level::trace;
    } else if (input == "DEBUG") {
        return spdlog::level::debug;
    } else if (input == "INFO") {
        return spdlog::level::info;
    } else if (input == "WARN") {
        return spdlog::level::warn;
    } else if (input == "ERROR") {
        return spdlog::level::err;
    } else if (input == "CRITICAL") {
        return spdlog::level::critical;
    } else {
        return -1;
    }
}

static void glfw_error_callback(int error, const char* description)
{
    mainLogger->error("GLFW Error %d: %s\n", error, description);
}

GLFWwindow* SetupWindow()
{

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        std::exit(-1);

    // GL ES 3.0 + GLSL 300 es (WebGL 2.0)
    const char* glsl_version = "#version 300 es";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    mainLogger->debug("Correct glfw version is set");

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Dear ImGui GLFW+OpenGL3 example", nullptr, nullptr);
    if (window == nullptr) {
        mainLogger->error("GLFW window couldn't be initialized. exiting.");
        std::exit(-1);
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImNodes::CreateContext(); // TODO this is kind  of spaghetti
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad; // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    mainLogger->info("Window setup complete");

    return window;
}

void CleanUp(GLFWwindow* window)
{
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImNodes::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}

void GuiLoop(GLFWwindow* window)
{
    Rendering::Graph graph;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetWindowAttrib(window, GLFW_ICONIFIED) != 0) {
            ImGui_ImplGlfw_Sleep(10);
            continue;
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        //-------------------------------------------Custom Code-------------------------------------------------------

        graph.RenderGraph();

        //-------------------------------------------Custom Code-------------------------------------------------------
        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }
}

void InitParserArgumnets(argparse::ArgumentParser& argparser)
{
    argparser.add_argument("--log-level")
        .help("Define the log-level of the program. Defuaults to INFO."
              "Allowed values: [TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL]")
        .default_value("INFO");
}

void ParseArguments(argparse::ArgumentParser& argparser, int argc, char* argv[])
{
    try {
        argparser.parse_args(argc, argv);
    } catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::exit(-1);
    }

    {
        auto inputLogLevel = argparser.get<std::string>("--log-level");
        auto logLevel = ConvertLogLevel(inputLogLevel);

        if (logLevel == -1) {
            std::cerr << std::format("Invalid log level input: {}, "
                                     "Allowed values: [TRACE, DEBUG, INFO, WARN, ERROR, CRITICAL]",
                inputLogLevel)
                      << std::endl;
            std::exit(-1);
        }

        // Sets GLOBAL log level
        spdlog::set_level(static_cast<spdlog::level::level_enum>(logLevel));
    }
}

int main(int argc, char* argv[])
{
    argparse::ArgumentParser argparser("2D Geometry Constraint Solver", "0.0.0");
    InitParserArgumnets(argparser);
    ParseArguments(argparser, argc, argv);

    GLFWwindow* window = SetupWindow();

    GuiLoop(window);

    CleanUp(window);

    return 0;
}

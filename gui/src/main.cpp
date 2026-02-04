// Custom headers
#include "application.hpp"

int main(int argc, char* argv[])
{
    auto app = Gui::Application::create();
    return app->run(argc, argv);
}

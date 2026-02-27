#include "main_window.hpp"

#include "model_serializer.hpp"

// General STD/STL headers
#include <fstream>
#include <sstream>
#include <string>

// Thirdparty headers
#include <spdlog/spdlog.h>

namespace Gui {

namespace {

    const auto MAIN_WINDOW_LOGGER = spdlog::stdout_color_mt("MAIN_WINDOW");

} // namespace

MainWindow::MainWindow()
    : m_modellerView(m_model)
    , m_decompositionView(m_model)
{
    set_title("2D Geometry Constraint Solver");
    set_default_size(1280, 800);

    // Header bar with view switcher
    m_stackSwitcher.set_stack(m_stack);
    m_headerBar.set_title_widget(m_stackSwitcher);

    // Save / Load buttons on the left side of the header bar
    m_saveBtn.set_label("Save");
    m_saveBtn.set_tooltip_text("Save the current model to a file");
    m_saveBtn.signal_clicked().connect(
        sigc::mem_fun(*this, &MainWindow::onSaveClicked));

    m_loadBtn.set_label("Load");
    m_loadBtn.set_tooltip_text("Load a model from a file");
    m_loadBtn.signal_clicked().connect(
        sigc::mem_fun(*this, &MainWindow::onLoadClicked));

    m_headerBar.pack_start(m_saveBtn);
    m_headerBar.pack_start(m_loadBtn);

    set_titlebar(m_headerBar);

    // Add views to the stack
    m_stack.add(m_modellerView, "modeller", "Modeller");
    m_stack.add(m_decompositionView, "decomposition", "Decomposition View");

    m_stack.set_transition_type(Gtk::StackTransitionType::SLIDE_LEFT_RIGHT);

    set_child(m_stack);
}

// ============================================================
// Save
// ============================================================

void MainWindow::onSaveClicked()
{
    auto dialog = Gtk::FileDialog::create();
    dialog->set_title("Save Model");

    // Set up file filter for .gcs files.
    auto filter = Gtk::FileFilter::create();
    filter->set_name("GCS Model Files (*.gcs)");
    filter->add_pattern("*.gcs");

    auto allFilter = Gtk::FileFilter::create();
    allFilter->set_name("All Files");
    allFilter->add_pattern("*");

    auto filters = Gio::ListStore<Gtk::FileFilter>::create();
    filters->append(filter);
    filters->append(allFilter);
    dialog->set_filters(filters);
    dialog->set_default_filter(filter);

    dialog->save(
        *this, [this, dialog](const Glib::RefPtr<Gio::AsyncResult>& result) {
            try {
                auto file = dialog->save_finish(result);
                if (!file) {
                    return;
                }

                std::string path = file->get_path();

                // Ensure .gcs extension if not already present.
                if (path.size() < 4 || path.substr(path.size() - 4) != ".gcs") {
                    path += ".gcs";
                }

                auto json
                    = ModelSerializer::serialize(m_modellerView.getCanvas());
                if (!json.has_value()) {
                    MAIN_WINDOW_LOGGER->error("Save failed: {}", json.error());
                    return;
                }

                writeFile(path, json.value());

            } catch (const Gtk::DialogError& e) {
                // User cancelled the dialog — not an error.
                if (e.code() != Gtk::DialogError::DISMISSED) {
                    MAIN_WINDOW_LOGGER->error(
                        "Save dialog error: {}", e.what());
                }
            }
        });
}

// ============================================================
// Load
// ============================================================

void MainWindow::onLoadClicked()
{
    auto dialog = Gtk::FileDialog::create();
    dialog->set_title("Load Model");

    // Set up file filter for .gcs files.
    auto filter = Gtk::FileFilter::create();
    filter->set_name("GCS Model Files (*.gcs)");
    filter->add_pattern("*.gcs");

    auto allFilter = Gtk::FileFilter::create();
    allFilter->set_name("All Files");
    allFilter->add_pattern("*");

    auto filters = Gio::ListStore<Gtk::FileFilter>::create();
    filters->append(filter);
    filters->append(allFilter);
    dialog->set_filters(filters);
    dialog->set_default_filter(filter);

    dialog->open(
        *this, [this, dialog](const Glib::RefPtr<Gio::AsyncResult>& result) {
            try {
                auto file = dialog->open_finish(result);
                if (!file) {
                    return;
                }

                readAndLoadFile(file->get_path());

            } catch (const Gtk::DialogError& e) {
                if (e.code() != Gtk::DialogError::DISMISSED) {
                    MAIN_WINDOW_LOGGER->error(
                        "Load dialog error: {}", e.what());
                }
            }
        });
}

// ============================================================
// File I/O helpers
// ============================================================

void MainWindow::writeFile(const std::string& path, const std::string& json)
{
    std::ofstream out(path);
    if (!out.is_open()) {
        MAIN_WINDOW_LOGGER->error("Failed to open file for writing: {}", path);
        return;
    }

    out << json;
    out.close();

    if (out.fail()) {
        MAIN_WINDOW_LOGGER->error("Failed to write to file: {}", path);
        return;
    }

    MAIN_WINDOW_LOGGER->info("Model saved to: {}", path);
}

void MainWindow::readAndLoadFile(const std::string& path)
{
    std::ifstream in(path);
    if (!in.is_open()) {
        MAIN_WINDOW_LOGGER->error("Failed to open file for reading: {}", path);
        return;
    }

    std::ostringstream buffer;
    buffer << in.rdbuf();

    if (in.fail() && !in.eof()) {
        MAIN_WINDOW_LOGGER->error("Failed to read file: {}", path);
        return;
    }

    auto modelData = ModelSerializer::deserialize(buffer.str());
    if (!modelData.has_value()) {
        MAIN_WINDOW_LOGGER->error(
            "Failed to deserialize model: {}", modelData.error());
        return;
    }

    // Clear the existing model state before loading.
    m_model.clear();

    auto error = m_modellerView.loadModelData(modelData.value());
    if (!error.empty()) {
        MAIN_WINDOW_LOGGER->error("Failed to load model data: {}", error);
        return;
    }

    MAIN_WINDOW_LOGGER->info("Model loaded from: {}", path);
}

} // namespace Gui

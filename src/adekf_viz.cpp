//
// Created by tomlucas on 07.04.20.
//

#include "adekf_viz.h"
#include "LinePlot.h"
namespace adekf::viz{
    void initGuis(int &argc, char *argv[]) {
        qwidget = std::make_shared<QApplication>(argc, argv);
        PoseRenderer::initGui();
    }
    void finishGuis() {
        PoseRenderer::disposeWindow();
        HeatMap::disposePlots();
        LinePlot::disposePlots();
    }
    void runGuis() {
        while (!PoseRenderer::isDone()) {
            PoseRenderer::updateWindow();
            HeatMap::updatePlots();
            LinePlot::ioService.poll();
            LinePlot::updatePlots();
            qwidget->processEvents();
        }
        finishGuis();
    }
    void plotVector(const Eigen::VectorXd &vector, const char *title, size_t buffer_size, const char *legend,size_t stride) {
        LinePlot::plotVector(vector, title, buffer_size, legend,stride);
    }

}
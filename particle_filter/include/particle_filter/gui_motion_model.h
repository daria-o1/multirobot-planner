#pragma once

#include <functional>  // For std::function.
#include <string>
#include <vector>

#include <vtkActor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkLookupTable.h>
#include <vtkPNGReader.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkGlyph3D.h>

#include <particle_filter/typedefs.h>

namespace particle_filter
{

class GuiMotionModel
{
  public:

    using MotionModelFunction = std::function<RobotPosition(RobotPosition robot_previous, RobotPosition robot_current, RobotPosition particle_position)>;
    using ParticleVector = std::vector<Particle>;

    GuiMotionModel(bool arrow = true);

    /** Draw and pause. */
    void startInteractor();

    /** Show the motion model for a predefined move
     */
    void displayMotionModel(MotionModelFunction motion_model);

    void screenshot(const std::string & filename);

  private:

    void makeGlyphs(vtkSmartPointer<vtkPolyData> src, double size, vtkSmartPointer<vtkGlyph3D> glyph, bool arrow = true);
    void setRobot(RobotPosition pos);
    void setParticles(ParticleVector particles, bool with_colormap, double size);
    void clearRobotPosition();

    /* vtkSmartPointer<vtkPolyData> map; */
    /* vtkSmartPointer<vtkVertexGlyphFilter> mapFilter; */
    /* vtkSmartPointer<vtkPolyDataMapper> mapMapper; */
    /* vtkSmartPointer<vtkActor> mapActor; */

    vtkSmartPointer<vtkPolyData> particle_positions_;
    /* vtkSmartPointer<vtkVertexGlyphFilter> particlesFilter; */
    /* vtkSmartPointer<vtkPolyDataMapper> particlesMapper; */
    vtkSmartPointer<vtkActor> particles_actor_;

    /* vtkSmartPointer<vtkPolyData> scanMeasurement; */
    /* vtkSmartPointer<vtkVertexGlyphFilter> scanFilter; */
    /* vtkSmartPointer<vtkPolyDataMapper> scanMapper; */
    /* vtkSmartPointer<vtkActor> scanActor; */

    vtkSmartPointer<vtkPolyData> robot_position_;
    /* vtkSmartPointer<vtkVertexGlyphFilter> positionFilter; */
    /* vtkSmartPointer<vtkPolyDataMapper> positionMapper; */
    /* vtkSmartPointer<vtkActor> positionActor; */

    vtkSmartPointer<vtkRenderer> renderer_;
    vtkSmartPointer<vtkRenderWindow> render_window_;
    vtkSmartPointer<vtkRenderWindowInteractor> render_window_interactor_;
    vtkSmartPointer<vtkInteractorStyleImage> interactor_style_;

    void vtkPointsFromRobotPosition(RobotPosition pos, vtkSmartPointer<vtkPoints> vtk_pts);
    void vtkPointsFromParticles(ParticleVector particles, bool with_colormap, vtkSmartPointer<vtkPoints> vtkPts, vtkSmartPointer<vtkUnsignedCharArray> vtkColors);

};

} /* namespace particle_filter */

#include <cmath>  // For std::round().
#include <cstdint>  // For std::size_t.
#include <iostream>

#include <vtkImageData.h>
#include <vtkPolyDataMapper.h>
#include <vtkDoubleArray.h>
#include <vtkPNGWriter.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkProperty.h>
#include <vtkTextureMapToPlane.h>
#include <vtkVersion.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPoints.h>
#include <vtkGlyph3D.h>
#include <vtkSmartPointer.h>
#include <vtkArrowSource.h>
#include <vtkDiskSource.h>

#include <particle_filter/gui_motion_model.h>

namespace particle_filter
{

/** Return min and max values, without considering values outside [0, 1].
*/
void normalizeProbabilities(ParticleVector particles, double & min, double & max)
{
  min = 1.0;
  max = 0.0;
  for (auto p : particles)
  {
    if ((p.weight < 0) or (p.weight > 1))
    {
      continue;
    }
    if (min > p.weight)
    {
      min = p.weight;
    }
    if (max < p.weight)
    {
      max = p.weight;
    }
  }
  if (min > max)
  {
    // Problem with input, set values that don't make troubles with the lookup table.
    min = 0.0;
    max = 1.0;
  }
}

GuiMotionModel::GuiMotionModel(bool arrow)
{
  particle_positions_ = vtkSmartPointer<vtkPolyData>::New();
  particle_positions_->SetPoints(vtkSmartPointer<vtkPoints>::New());

  robot_position_ = vtkSmartPointer<vtkPolyData>::New();
  robot_position_->SetPoints(vtkSmartPointer<vtkPoints>::New());

  //   particlesFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  // #if VTK_MAJOR_VERSION <= 5
  //   particlesFilter->AddInput(particles);
  // #else
  //   particlesFilter->AddInputData(particles);
  // #endif
  //   particlesFilter->Update();

  //   particlesMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  //   particlesMapper->SetInputConnection(particlesFilter->GetOutputPort());

  //   particlesActor = vtkSmartPointer<vtkActor>::New();
  //   particlesActor->SetMapper(particlesMapper);
  //   particlesActor->GetProperty()->SetColor(1, 0, 0);
  //   particlesActor->GetProperty()->SetPointSize(3);

  vtkSmartPointer<vtkGlyph3D> glyph_3d{vtkSmartPointer<vtkGlyph3D>::New()};
  makeGlyphs(particle_positions_, 0.02, glyph_3d, arrow);

  vtkSmartPointer<vtkPolyDataMapper> glyph_3d_mapper{vtkSmartPointer<vtkPolyDataMapper>::New()};
  glyph_3d_mapper->SetInputConnection(glyph_3d->GetOutputPort());
  particles_actor_ = vtkSmartPointer<vtkActor>::New();
  particles_actor_->SetMapper(glyph_3d_mapper);
  particles_actor_->GetProperty()->SetColor(1.0, 0.0, 0.0);

  //   positionFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
  // #if VTK_MAJOR_VERSION <= 5
  //   positionFilter->AddInput(position);
  // #else
  //   positionFilter->AddInputData(position);
  // #endif
  //   positionFilter->Update();

  //   positionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  //   positionMapper->SetInputConnection(positionFilter->GetOutputPort());

  //   positionActor = vtkSmartPointer<vtkActor>::New();
  //   positionActor->SetMapper(positionMapper);
  //   positionActor->GetProperty()->SetColor(0, 1, 0);
  //   positionActor->GetProperty()->SetPointSize(5);

  vtkSmartPointer<vtkGlyph3D> position_glyph{vtkSmartPointer<vtkGlyph3D>::New()};
  makeGlyphs(robot_position_, 0.1, position_glyph, arrow);

  vtkSmartPointer<vtkPolyDataMapper> position_glyph_mapper{vtkSmartPointer<vtkPolyDataMapper>::New()};
  position_glyph_mapper->SetInputConnection(position_glyph->GetOutputPort());
  vtkSmartPointer<vtkActor> position_glyph_actor{vtkSmartPointer<vtkActor>::New()};
  position_glyph_actor->SetMapper(position_glyph_mapper);
  position_glyph_actor->GetProperty()->SetColor(0.3400, 0.8900, 0.8100);

  renderer_ = vtkSmartPointer<vtkRenderer>::New();
  render_window_ = vtkSmartPointer<vtkRenderWindow>::New();
  render_window_->SetSize(1400, 1050);
  render_window_->AddRenderer(renderer_);

  render_window_interactor_ = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  render_window_interactor_->SetRenderWindow(render_window_);

  interactor_style_ = vtkSmartPointer<vtkInteractorStyleImage>::New();
  render_window_interactor_->SetInteractorStyle(interactor_style_);

  //   renderer_->AddActor(particlesActor);
  //   renderer_->AddActor(positionActor);
  renderer_->AddActor(position_glyph_actor);
  renderer_->AddActor(particles_actor_);
  renderer_->SetBackground(0.15, 0.15, 0.15);
  render_window_->Render();
}

void GuiMotionModel::makeGlyphs(vtkSmartPointer<vtkPolyData> src, double size, vtkSmartPointer<vtkGlyph3D> glyph, bool arrow)
{
  // Source for the glyph filter.
  if (arrow)
  {
    vtkSmartPointer<vtkArrowSource> arrow{vtkSmartPointer<vtkArrowSource>::New()};
    arrow->SetTipResolution(1.0);
    arrow->SetShaftResolution(1.0);
    arrow->SetTipLength(0.4);
    arrow->SetTipRadius(0.1);
    arrow->SetShaftRadius(0.03);
    glyph->SetSourceConnection(arrow->GetOutputPort());
  }
  else
  {
    vtkSmartPointer<vtkDiskSource> disc{vtkSmartPointer<vtkDiskSource>::New()};
    disc->SetInnerRadius(0.0);
    disc->SetOuterRadius(0.1);
    glyph->SetSourceConnection(disc->GetOutputPort());
  }

  glyph->SetInputData(src);
  glyph->SetVectorModeToUseNormal();
  glyph->SetScaleModeToScaleByVector();
  glyph->SetScaleFactor(size);
  glyph->OrientOn();
  glyph->Update();
}

void GuiMotionModel::vtkPointsFromRobotPosition(RobotPosition pos, vtkSmartPointer<vtkPoints> vtk_pts)
{
  vtk_pts->SetNumberOfPoints(1);
  vtk_pts->SetPoint(0, pos.x, pos.y, 0.0);
}

void GuiMotionModel::vtkPointsFromParticles(ParticleVector particles, bool with_colormap, vtkSmartPointer<vtkPoints> vtkPts, vtkSmartPointer<vtkUnsignedCharArray> vtkColors)
{
  // vtkPts->Reset() and vtkColors->Reset() segfault, so this is up to the
  // caller to ensure that vtkPts and vtkColors are empty or have at least
  // the same number of elements.

  vtkSmartPointer<vtkLookupTable> colorLookupTable;
  if (with_colormap)
  {
    // Normalize the colormap.
    colorLookupTable = vtkSmartPointer<vtkLookupTable>::New();
    double min;
    double max;
    normalizeProbabilities(particles, min, max);
    colorLookupTable->SetTableRange(min, max);
    colorLookupTable->Build(); 
  }

  vtkColors->SetNumberOfComponents(3);
  for (auto p : particles)
  {
    vtkPts->InsertNextPoint(p.pos.x, p.pos.y, 0.0);
    double color[3];  // Color within [0, 1].
    if (with_colormap)
    {
      if ((p.weight < 0) || (p.weight > 1))
      {
        // Mark a probability outside [0, 1] as grey.
        color[0] = 0.6;
        color[1] = 0.6;
        color[2] = 0.6;
      }
      else
      {
        colorLookupTable->GetColor(p.weight, color);
      }
    }
    else
    {
      // Uniform color: red.
      color[0] = 1.0;
      color[1] = 0.0;
      color[2] = 0.0;
    }
    unsigned char color_i[3];  // Color within [0, 255].
    for (size_t c = 0; c < 3; c++)
    {
      color_i[c] = static_cast<unsigned char>(std::round(color[c] * 255));
    }
    vtkColors->InsertNextTupleValue(color_i);
  }
}

void GuiMotionModel::startInteractor()
{
  render_window_interactor_->Start();
}

void GuiMotionModel::setParticles(ParticleVector particles, bool with_colormap, double size)
{
  vtkSmartPointer<vtkPoints> points{vtkSmartPointer<vtkPoints>::New()};
  vtkSmartPointer<vtkUnsignedCharArray> colors{vtkSmartPointer<vtkUnsignedCharArray>::New()};
  colors->SetNumberOfComponents(3);
  vtkPointsFromParticles(particles, with_colormap, points, colors);

  particle_positions_->SetPoints(points);
  particle_positions_->GetPointData()->SetScalars(colors);

  /* Set point normals. */
  vtkSmartPointer<vtkDoubleArray> point_normals_array{vtkSmartPointer<vtkDoubleArray>::New()};
  point_normals_array->SetNumberOfComponents(3);  // 3d normals (ie x,y,z)
  point_normals_array->SetNumberOfTuples(particles.size());

  for (size_t i = 0; i < particles.size(); ++i)
  {
    double normal[3] = {std::cos(particles[i].pos.phi), std::sin(particles[i].pos.phi), 0.0};
    point_normals_array->SetTuple(i, normal);
  }
  particle_positions_->GetPointData()->SetNormals(point_normals_array);

  particles_actor_->GetProperty()->SetPointSize(size);

  render_window_->Render();
}

void GuiMotionModel::clearRobotPosition()
{
  robot_position_->SetPoints(vtkSmartPointer<vtkPoints>::New());
  robot_position_->Modified();
  render_window_->Render();
}

void GuiMotionModel::setRobot(RobotPosition pos)
{
  vtkSmartPointer<vtkPoints> p{robot_position_->GetPoints()};
  vtkPointsFromRobotPosition(pos, p);

  robot_position_->Modified();

  render_window_->Render();
}

void GuiMotionModel::displayMotionModel(MotionModelFunction motion_model)
{
  constexpr size_t particle_count{100};

  RobotPosition previous{0.0, 0.0, 0.0};
  RobotPosition current{1.0, 0.1, 0.2};

  /* Apply the motion model to all particles. */
  ParticleVector particles;
  particles.reserve(particle_count);
  for (std::size_t i = 0; i < particle_count; ++i)
  {
    Particle p{previous, 1.0 / static_cast<double>(particle_count)};
    Particle new_p{motion_model(previous, current, p.pos), p.weight};
    particles.push_back(new_p);
  }

  /* Show the robot position and the particles. */
  setRobot(previous);
  setParticles(particles, false, 3.0);
  startInteractor();
}
 
void GuiMotionModel::screenshot(const std::string& filename)
{
  renderer_->ResetCamera();
  render_window_->Render();

  vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter{vtkSmartPointer<vtkWindowToImageFilter>::New()};
  window_to_image_filter->SetInput(render_window_);
  // Set the resolution of the output image (5 times the current resolution
  // of vtk render window).
  window_to_image_filter->SetMagnification(5);
  // Also record the alpha (transparency) channel.
  window_to_image_filter->SetInputBufferTypeToRGBA();
  window_to_image_filter->ReadFrontBufferOff();
  window_to_image_filter->Update();

  vtkSmartPointer<vtkPNGWriter> writer{vtkSmartPointer<vtkPNGWriter>::New()};
  writer->SetFileName(filename.c_str());
  writer->SetInputConnection(window_to_image_filter->GetOutputPort());
  writer->Write();
}

} /* namespace particle_filter */

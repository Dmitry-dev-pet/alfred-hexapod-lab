import vtk,os,sys
path=sys.argv[1];target=sys.argv[2]
print('VTK',vtk.vtkVersion.GetVTKVersion(),flush=True)
win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(1080,800)
ren=vtk.vtkRenderer();win.AddRenderer(ren)
imp=vtk.vtkGLTFImporter();imp.SetFileName(path);imp.SetRenderWindow(win);imp.Update()
print('actors',ren.GetActors().GetNumberOfItems(),flush=True)
ren.SetBackground(0.9,0.91,0.94)
ren.ResetCamera();cam=ren.GetActiveCamera();cam.SetFocalPoint(0,0,.12);cam.SetPosition(.48,-.48,.32);cam.SetViewUp(0,0,1);cam.SetClippingRange(.01,10);ren.ResetCameraClippingRange()
light=vtk.vtkLight();light.SetLightTypeToSceneLight();light.SetPosition(.3,-.8,1);light.SetFocalPoint(0,0,.1);light.SetIntensity(1.8);ren.AddLight(light)
win.Render();f=vtk.vtkWindowToImageFilter();f.SetInput(win);f.SetInputBufferTypeToRGB();f.ReadFrontBufferOff();f.Update();wr=vtk.vtkPNGWriter();wr.SetInputData(f.GetOutput());wr.SetFileName(target);wr.Write()
print('SAVED',target,os.path.getsize(target),flush=True)

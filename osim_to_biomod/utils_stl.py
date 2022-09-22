import vtk
from pathlib import Path


def stl_to_vtp(input_filename, output_filename):
    reader_stl = vtk.vtkSTLReader()
    reader_stl.SetFileName(input_filename)
    # TODO: Clean up all these comments
    # 'update' the reader i.e. read the .stl file
    reader_stl.Update()
    polydata = reader_stl.GetOutput()

    writer = vtk.vtkXMLPolyDataWriter()
    writer.SetFileName(output_filename)
    writer.SetInputData(polydata)
    writer.SetDataModeToAscii()

    # writer.SetFileTypeToASCII()
    writer.Write()


def scale_stl(filename, scale=(1, 1, 1), output_filename=None):
    output_filename = output_filename if output_filename else f"{Path(filename).stem}_scaled.stl"

    reader = vtk.vtkSTLReader()
    reader.SetFileName(filename)
    reader.Update()

    transform = vtk.vtkTransform()
    transform.Scale(scale)

    transform_filter = vtk.vtkTransformPolyDataFilter()
    transform_filter.SetInputConnection(reader.GetOutputPort())
    transform_filter.SetTransform(transform)
    transform_filter.Update()

    # TODO: Including there (remove or uncomment)
    # sim = vtk.vtkDecimatePro()
    # sim.SetTargetReduction(5)
    # sim.SetInputData(reader.GetOutputPort())
    # sim.PreserveTopologyOn()

    # writer = vtk.vtkSTLWriter()
    # writer.SetFileName(output_filename)
    # writer.SetInputData(transform_filter.GetOutput())
    # writer.Write()

    writer = vtk.vtkXMLPolyDataWriter()
    writer.SetFileName(output_filename)
    writer.SetInputData(transform_filter.GetOutput())
    writer.SetDataModeToAscii()

    # writer.SetFileTypeToASCII()
    writer.Write()


def reduce_stl(filename, ratio=1.0, output_filename=None):
    output_filename = output_filename if output_filename else f"{Path(filename).stem}_reduced.stl"

    reader = vtk.vtkSTLReader()
    reader.SetFileName(filename)
    reader.Update()

    triangles = vtk.vtkTriangleFilter()
    triangles.SetInputData(reader.GetOutput())
    triangles.Update()

    decimate = vtk.vtkQuadricDecimation()
    decimate.SetInputData(triangles.GetOutput())
    decimate.SetTargetReduction(ratio)
    # decimate.PreserveTopologyOn()
    decimate.Update()

    writer = vtk.vtkSTLWriter()
    writer.SetFileName(output_filename)
    writer.SetInputData(decimate.GetOutput())
    writer.Write()

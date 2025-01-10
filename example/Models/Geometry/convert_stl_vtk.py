import vtk
from pathlib import Path


def stl2vtp(input_filename, output_filename):
    readerSTL = vtk.vtkSTLReader()
    readerSTL.SetFileName(input_filename)
    # 'update' the reader i.e. read the .stl file
    readerSTL.Update()
    polydata = readerSTL.GetOutput()

    writter = vtk.vtkXMLPolyDataWriter()
    writter.SetFileName(output_filename)
    writter.SetInputData(polydata)
    writter.SetDataModeToAscii()

    # writter.SetFileTypeToASCII()
    writter.Write()


def scalestl(filename, scale=(1, 1, 1), output_filename=None):
    output_filename = output_filename if output_filename else f"{Path(filename).stem}_scaled.stl"

    reader = vtk.vtkSTLReader()
    reader.SetFileName(filename)
    reader.Update()

    transform = vtk.vtkTransform()
    transform.Scale(scale)

    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetInputConnection(reader.GetOutputPort())
    transformFilter.SetTransform(transform)
    transformFilter.Update()

    # sim = vtk.vtkDecimatePro()
    # sim.SetTargetReduction(5)
    # sim.SetInputData(reader.GetOutputPort())
    # sim.PreserveTopologyOn()

    # writer = vtk.vtkSTLWriter()
    # writer.SetFileName(output_filename)
    # writer.SetInputData(transformFilter.GetOutput())
    # writer.Write()

    writter = vtk.vtkXMLPolyDataWriter()
    writter.SetFileName(output_filename)
    writter.SetInputData(transformFilter.GetOutput())
    writter.SetDataModeToAscii()

    # writter.SetFileTypeToASCII()
    writter.Write()


def reducestl(filename, ratio=1.0, output_filename=None):
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


if __name__ == "__main__":
    # stl2vtp(input_filename="prothese.stl", output_filename="prothese.vtp")
    filename = "humerus.stl"
    scalestl(filename, (0.001, 0.001, 0.001), output_filename="/osim_model/Geometry/humerus_scaled.vtp")
    # filename = "scapula_wt_aromion_scaled.stl"
    # scalestl(filename, (0.114, 0.114, 0.114))
    # reducestl(filename, ratio=2)

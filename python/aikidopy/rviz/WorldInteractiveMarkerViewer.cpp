#ifdef AIKIDO_HAS_RVIZ

#include <pybind11/pybind11.h>
#include <aikido/rviz.hpp>

namespace py = pybind11;

namespace aikido {
namespace python {

void WorldInteractiveMarkerViewer(py::module& m)
{
    py::class_<aikido::rviz::WorldInteractiveMarkerViewer, std::shared_ptr<aikido::rviz::WorldInteractiveMarkerViewer>>(m, "WorldInteractiveMarkerViewer")
      .def("addTSRMarker",
        [](aikido::rviz::WorldInteractiveMarkerViewer* self,
          std::shared_ptr<aikido::constraint::dart::TSR> tsr)
        -> aikido::rviz::TSRMarkerPtr
        {
          return self->addTSRMarker(*tsr.get());
        });
}

} // namespace python
} // namespace aikido

#endif // AIKIDO_HAS_RVIZ

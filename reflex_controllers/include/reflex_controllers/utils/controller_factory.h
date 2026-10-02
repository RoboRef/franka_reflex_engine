#pragma once

#include <reflex_controllers/base/panda_controller_interface.h>
#include <reflex_controllers/utils/factory.h>

namespace panda_controllers {

using ControllerFactory = Factory<PandaControllerInterface>;

} // namespace panda_controllers

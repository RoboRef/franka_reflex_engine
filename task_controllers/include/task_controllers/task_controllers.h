#pragma once

namespace task_controllers {
  // Calling this function forces the dynamic linker to load libtask_controllers.so,
  // executing all static ControllerFactory::registerClass registrations.
  void init();
}

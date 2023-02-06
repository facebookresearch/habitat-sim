# Helpers nvp

Table of Contents

- [include_gl.h](#include_glh)
- [NvFoundation.h](#NvFoundationh)
- [nvpsystem.hpp](#nvpsystemhpp)
- [nvpwindow.hpp](#nvpwindowhpp)
- [platform.h](#platformh)
- [resources.h](#resourcesh)
_____

# nvpsystem.hpp

<a name="nvpsystemhpp"></a>
## class NVPSystem
> NVPSystem is a utility class to handle some basic system
functionality that all projects likely make use of.

It does not require any window to be opened.
Typical usage is calling init right after main and deinit
in the end, or use the NVPSystem object for that.
init
- calls glfwInit and registers the error callback for it
- sets up and log filename based on projectName via nvprintSetLogFileName
- if NVP_SUPPORTS_SOCKETS is set, starts socket server as well



_____

# nvpwindow.hpp

<a name="nvpwindowhpp"></a>
## class NVPWindow
> base class for a window, to catch events

Using and deriving of NVPWindow base-class is optional.
However one must always make use of the NVPSystem
That takes care of glfwInit/terminate as well.

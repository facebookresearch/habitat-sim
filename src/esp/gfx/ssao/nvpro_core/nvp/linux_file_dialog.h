//
//  Courtesy from Sam Hocevar <sam@hocevar.net>
//

/// \nodoc (keyword to exclude this file from automatic README.md generation)

#pragma once

#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 2  // for popen()
#endif
#include <fcntl.h>     // fcntl()
#include <sys/wait.h>  // waitpid()
#include <unistd.h>    // read(), pipe(), dup2()
#include <csignal>     // ::kill, std::signal
#include <cstdio>      // popen()
#include <cstdlib>     // std::getenv()

#include <chrono>    // std::chrono
#include <iostream>  // std::ostream
#include <map>       // std::map
#include <memory>    // std::shared_ptr
#include <regex>     // std::regex
#include <set>       // std::set
#include <string>    // std::string
#include <thread>    // std::mutex, std::this_thread

enum class button {
  cancel = -1,
  ok,
  yes,
  no,
  abort,
  retry,
  ignore,
};

enum class choice {
  ok = 0,
  ok_cancel,
  yes_no,
  yes_no_cancel,
  retry_cancel,
  abort_retry_ignore,
};

enum class icon {
  info = 0,
  warning,
  error,
  question,
};

// Additional option flags for various dialog constructors
enum class opt : uint8_t {
  none = 0,
  // For file open, allow multiselect.
  multiselect = 0x1,
  // For file save, force overwrite and disable the confirmation dialog.
  force_overwrite = 0x2,
  // For folder select, force path to be the provided argument instead
  // of the last opened directory, which is the Microsoft-recommended,
  // user-friendly behaviour.
  force_path = 0x4,
};

inline opt operator|(opt a, opt b) {
  return opt(uint8_t(a) | uint8_t(b));
}
inline bool operator&(opt a, opt b) {
  return bool(uint8_t(a) & uint8_t(b));
}

// The settings class, only exposing to the user a way to set verbose mode
// and to force a rescan of installed desktop helpers (zenity, kdialog…).
class settings {
 public:
  static bool available();

  static void verbose(bool value);
  static void rescan();

 protected:
  explicit settings(bool resync = false);

  bool check_program(std::string const& program);

  inline bool is_zenity() const;
  inline bool is_kdialog() const;

  enum class flag {
    is_scanned = 0,
    is_verbose,

    has_zenity,
    has_matedialog,
    has_qarma,
    has_kdialog,
    is_vista,

    max_flag,
  };

  // Static array of flags for internal state
  bool const& flags(flag in_flag) const;

  // Non-const getter for the static array of flags
  bool& flags(flag in_flag);
};

// Internal classes, not to be used by client applications
namespace internal {

// Process wait timeout, in milliseconds
static int const default_wait_timeout = 20;

class executor {
  friend class dialog;

 public:
  // High level function to get the result of a command
  std::string result(int* exit_code = nullptr);

  // High level function to abort
  bool kill();

  void start_process(std::vector<std::string> const& command);

  ~executor();

 protected:
  bool ready(int timeout = default_wait_timeout);
  void stop();

 private:
  bool m_running = false;
  std::string m_stdout;
  int m_exit_code = -1;
  pid_t m_pid = 0;
  int m_fd = -1;
};

class dialog : protected settings {
 public:
  bool ready(int timeout = default_wait_timeout) const;
  bool kill() const;

 protected:
  explicit dialog();

  std::vector<std::string> desktop_helper() const;
  static std::string buttons_to_name(choice _choice);
  static std::string get_icon_name(icon _icon);

  std::string powershell_quote(std::string const& str) const;
  std::string shell_quote(std::string const& str) const;

  // Keep handle to executing command
  std::shared_ptr<executor> m_async;
};

class file_dialog : public dialog {
 protected:
  enum type {
    open,
    save,
    folder,
  };

  file_dialog(type in_type,
              std::string const& title,
              std::string const& default_path = "",
              std::vector<std::string> const& filters = {},
              opt options = opt::none);

 protected:
  std::string string_result();
  std::vector<std::string> vector_result();
};

}  // namespace internal

//
// The notify widget
//

class notify : public internal::dialog {
 public:
  notify(std::string const& title,
         std::string const& message,
         icon _icon = icon::info);
};

//
// The message widget
//

class message : public internal::dialog {
 public:
  message(std::string const& title,
          std::string const& text,
          choice _choice = choice::ok_cancel,
          icon _icon = icon::info);

  button result();

 private:
  // Some extra logic to map the exit code to button number
  std::map<int, button> m_mappings;
};

//
// The open_file, save_file, and open_folder widgets
//

class open_file : public internal::file_dialog {
 public:
  open_file(std::string const& title,
            std::string const& default_path = "",
            std::vector<std::string> const& filters = {"All Files", "*"},
            opt options = opt::none);

#if defined(__has_cpp_attribute)
#if __has_cpp_attribute(deprecated)
  // Backwards compatibility
  [[deprecated("Use opt::multiselect instead of allow_multiselect")]]
#endif
#endif
  open_file(std::string const& title,
            std::string const& default_path,
            std::vector<std::string> const& filters,
            bool allow_multiselect);

  std::vector<std::string> result();
};

class save_file : public internal::file_dialog {
 public:
  save_file(std::string const& title,
            std::string const& default_path = "",
            std::vector<std::string> const& filters = {"All Files", "*"},
            opt options = opt::none);

#if defined(__has_cpp_attribute)
#if __has_cpp_attribute(deprecated)
  // Backwards compatibility
  [[deprecated("Use opt::force_overwrite instead of confirm_overwrite")]]
#endif
#endif
  save_file(std::string const& title,
            std::string const& default_path,
            std::vector<std::string> const& filters,
            bool confirm_overwrite);

  std::string result();
};

class select_folder : public internal::file_dialog {
 public:
  select_folder(std::string const& title,
                std::string const& default_path = "",
                opt options = opt::none);

  std::string result();
};

//
// Below this are all the method implementations. You may choose to define the
// macro SKIP_IMPLEMENTATION everywhere before including this header except
// in one place. This may reduce compilation times.
//

#if !defined SKIP_IMPLEMENTATION

// internal free functions implementations

namespace internal {
// This is necessary until C++20 which will have std::string::ends_with() etc.

static inline bool ends_with(std::string const& str,
                             std::string const& suffix) {
  return suffix.size() <= str.size() &&
         str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

static inline bool starts_with(std::string const& str,
                               std::string const& prefix) {
  return prefix.size() <= str.size() &&
         str.compare(0, prefix.size(), prefix) == 0;
}

}  // namespace internal

// settings implementation

inline settings::settings(bool resync) {
  flags(flag::is_scanned) &= !resync;

  if (flags(flag::is_scanned))
    return;

  flags(flag::has_zenity) = check_program("zenity");
  flags(flag::has_matedialog) = check_program("matedialog");
  flags(flag::has_qarma) = check_program("qarma");
  flags(flag::has_kdialog) = check_program("kdialog");

  // If multiple helpers are available, try to default to the best one
  if (flags(flag::has_zenity) && flags(flag::has_kdialog)) {
    auto desktop_name = std::getenv("XDG_SESSION_DESKTOP");
    if (desktop_name && desktop_name == std::string("gnome"))
      flags(flag::has_kdialog) = false;
    else if (desktop_name && desktop_name == std::string("KDE"))
      flags(flag::has_zenity) = false;
  }

  flags(flag::is_scanned) = true;
}

inline bool settings::available() {
  settings tmp;
  return tmp.flags(flag::has_zenity) || tmp.flags(flag::has_matedialog) ||
         tmp.flags(flag::has_qarma) || tmp.flags(flag::has_kdialog);
}

inline void settings::verbose(bool value) {
  settings().flags(flag::is_verbose) = value;
}

inline void settings::rescan() {
  settings(/* resync = */ true);
}

// Check whether a program is present using “which”.
inline bool settings::check_program(std::string const& program) {
  int exit_code = -1;
  internal::executor async;
  async.start_process({"/bin/sh", "-c", "which " + program});
  async.result(&exit_code);
  return exit_code == 0;
}

inline bool settings::is_zenity() const {
  return flags(flag::has_zenity) || flags(flag::has_matedialog) ||
         flags(flag::has_qarma);
}

inline bool settings::is_kdialog() const {
  return flags(flag::has_kdialog);
}

inline bool const& settings::flags(flag in_flag) const {
  static bool flags[size_t(flag::max_flag)];
  return flags[size_t(in_flag)];
}

inline bool& settings::flags(flag in_flag) {
  return const_cast<bool&>(static_cast<settings const*>(this)->flags(in_flag));
}

// executor implementation

inline std::string internal::executor::result(int* exit_code /* = nullptr */) {
  stop();
  if (exit_code)
    *exit_code = m_exit_code;
  return m_stdout;
}

inline bool internal::executor::kill() {
  ::kill(m_pid, SIGKILL);
  stop();
  return true;
}

inline void internal::executor::start_process(
    std::vector<std::string> const& command) {
  stop();
  m_stdout.clear();
  m_exit_code = -1;

  int in[2], out[2];
  if (pipe(in) != 0 || pipe(out) != 0)
    return;

  m_pid = fork();
  if (m_pid < 0)
    return;

  close(in[m_pid ? 0 : 1]);
  close(out[m_pid ? 1 : 0]);

  if (m_pid == 0) {
    dup2(in[0], STDIN_FILENO);
    dup2(out[1], STDOUT_FILENO);

    // Ignore stderr so that it doesn’t pollute the console (e.g. GTK+ errors
    // from zenity)
    int fd = open("/dev/null", O_WRONLY);
    dup2(fd, STDERR_FILENO);
    close(fd);

    std::vector<char*> args;
    std::transform(
        command.cbegin(), command.cend(), std::back_inserter(args),
        [](std::string const& s) { return const_cast<char*>(s.c_str()); });
    args.push_back(nullptr);  // null-terminate argv[]

    execvp(args[0], args.data());
    exit(1);
  }

  close(in[1]);
  m_fd = out[0];
  auto flags = fcntl(m_fd, F_GETFL);
  fcntl(m_fd, F_SETFL, flags | O_NONBLOCK);

  m_running = true;
}

inline internal::executor::~executor() {
  stop();
}

inline bool internal::executor::ready(
    int timeout /* = default_wait_timeout */) {
  if (!m_running)
    return true;

  char buf[BUFSIZ];
  ssize_t received = read(m_fd, buf, BUFSIZ);  // Flawfinder: ignore
  if (received > 0) {
    m_stdout += std::string(buf, received);
    return false;
  }

  // Reap child process if it is dead. It is possible that the system has
  // already reaped it (this happens when the calling application handles or
  // ignores SIG_CHLD) and results in waitpid() failing with ECHILD. Otherwise
  // we assume the child is running and we sleep for a little while.
  int status;
  pid_t child = waitpid(m_pid, &status, WNOHANG);
  if (child != m_pid && (child >= 0 || errno != ECHILD)) {
    // FIXME: this happens almost always at first iteration
    std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
    return false;
  }

  close(m_fd);
  m_exit_code = WEXITSTATUS(status);

  m_running = false;
  return true;
}

inline void internal::executor::stop() {
  // Loop until the user closes the dialog
  while (!ready())
    ;
}

// dialog implementation

inline bool internal::dialog::ready(
    int timeout /* = default_wait_timeout */) const {
  return m_async->ready(timeout);
}

inline bool internal::dialog::kill() const {
  return m_async->kill();
}

inline internal::dialog::dialog() : m_async(std::make_shared<executor>()) {}

inline std::vector<std::string> internal::dialog::desktop_helper() const {
  return {flags(flag::has_zenity)       ? "zenity"
          : flags(flag::has_matedialog) ? "matedialog"
          : flags(flag::has_qarma)      ? "qarma"
          : flags(flag::has_kdialog)    ? "kdialog"
                                        : "echo"};
}

inline std::string internal::dialog::buttons_to_name(choice _choice) {
  switch (_choice) {
    case choice::ok_cancel:
      return "okcancel";
    case choice::yes_no:
      return "yesno";
    case choice::yes_no_cancel:
      return "yesnocancel";
    case choice::retry_cancel:
      return "retrycancel";
    case choice::abort_retry_ignore:
      return "abortretryignore";
    /* case choice::ok: */ default:
      return "ok";
  }
}

inline std::string internal::dialog::get_icon_name(icon _icon) {
  switch (_icon) {
    case icon::warning:
      return "warning";
    case icon::error:
      return "error";
    case icon::question:
      return "question";
    // Zenity wants "information" but WinForms wants "info"
    /* case icon::info: */ default:
      return "information";
  }
}

// THis is only used for debugging purposes
inline std::ostream& operator<<(std::ostream& s,
                                std::vector<std::string> const& v) {
  int not_first = 0;
  for (auto& e : v)
    s << (not_first++ ? " " : "") << e;
  return s;
}

// Properly quote a string for Powershell: replace ' or " with '' or ""
// FIXME: we should probably get rid of newlines!
// FIXME: the \" sequence seems unsafe, too!
// XXX: this is no longer used but I would like to keep it around just in case
inline std::string internal::dialog::powershell_quote(
    std::string const& str) const {
  return "'" + std::regex_replace(str, std::regex("['\"]"), "$&$&") + "'";
}

// Properly quote a string for the shell: just replace ' with '\''
// XXX: this is no longer used but I would like to keep it around just in case
inline std::string internal::dialog::shell_quote(std::string const& str) const {
  return "'" + std::regex_replace(str, std::regex("'"), "'\\''") + "'";
}

// file_dialog implementation

inline internal::file_dialog::file_dialog(
    type in_type,
    std::string const& title,
    std::string const& default_path /* = "" */,
    std::vector<std::string> const& filters /* = {} */,
    opt options /* = opt::none */) {
  auto command = desktop_helper();

  if (is_zenity()) {
    command.push_back("--file-selection");
    command.push_back("--filename=" + default_path);
    command.push_back("--title");
    command.push_back(title);
    command.push_back("--separator=\n");

    for (size_t i = 0; i < filters.size() / 2; ++i) {
      command.push_back("--file-filter");
      command.push_back(filters[2 * i] + "|" + filters[2 * i + 1]);
    }

    if (in_type == type::save)
      command.push_back("--save");
    if (in_type == type::folder)
      command.push_back("--directory");
    if (!(options & opt::force_overwrite))
      command.push_back("--confirm-overwrite");
    if (options & opt::multiselect)
      command.push_back("--multiple");
  } else if (is_kdialog()) {
    switch (in_type) {
      case type::save:
        command.push_back("--getsavefilename");
        break;
      case type::open:
        command.push_back("--getopenfilename");
        break;
      case type::folder:
        command.push_back("--getexistingdirectory");
        break;
    }
    if (options & opt::multiselect)
      command.push_back(" --multiple");

    command.push_back(default_path);

    std::string filter;
    for (size_t i = 0; i < filters.size() / 2; ++i)
      filter += (i == 0 ? "" : " | ") + filters[2 * i] + "(" +
                filters[2 * i + 1] + ")";
    command.push_back(filter);

    command.push_back("--title");
    command.push_back(title);
  }

  if (flags(flag::is_verbose))
    std::cerr << command << std::endl;

  m_async->start_process(command);
}

inline std::string internal::file_dialog::string_result() {
  auto ret = m_async->result();
  // Strip potential trailing newline (zenity). Also strip trailing slash.
  while (!ret.empty() && (ret.back() == '\n' || ret.back() == '/'))
    ret.pop_back();
  return ret;
}

inline std::vector<std::string> internal::file_dialog::vector_result() {
  std::vector<std::string> ret;
  auto result = m_async->result();
  for (;;) {
    // Split result along newline characters
    auto i = result.find('\n');
    if (i == 0 || i == std::string::npos)
      break;
    ret.push_back(result.substr(0, i));
    result = result.substr(i + 1, result.size());
  }
  return ret;
}

// notify implementation

inline notify::notify(std::string const& title,
                      std::string const& message,
                      icon _icon /* = icon::info */) {
  if (_icon == icon::question)  // Not supported by notifications
    _icon = icon::info;

  auto command = desktop_helper();

  if (is_zenity()) {
    command.push_back("--notification");
    command.push_back("--window-icon");
    command.push_back(get_icon_name(_icon));
    command.push_back("--text");
    command.push_back(title + "\n" + message);
  } else if (is_kdialog()) {
    command.push_back("--icon");
    command.push_back(get_icon_name(_icon));
    command.push_back("--title");
    command.push_back(title);
    command.push_back("--passivepopup");
    command.push_back(message);
    command.push_back("5");
  }

  if (flags(flag::is_verbose))
    std::cerr << command << std::endl;

  m_async->start_process(command);
}

// message implementation

inline message::message(std::string const& title,
                        std::string const& text,
                        choice _choice /* = choice::ok_cancel */,
                        icon _icon /* = icon::info */) {
  auto command = desktop_helper();
  if (is_zenity()) {
    switch (_choice) {
      case choice::ok_cancel:
        command.insert(command.end(), {"--question", "--cancel-label=Cancel",
                                       "--ok-label=OK"});
        break;
      case choice::yes_no:
        // Do not use standard --question because it causes “No” to return -1,
        // which is inconsistent with the “Yes/No/Cancel” mode below.
        command.insert(command.end(),
                       {"--question", "--switch", "--extra-button=No",
                        "--extra-button=Yes"});
        break;
      case choice::yes_no_cancel:
        command.insert(command.end(),
                       {"--question", "--switch", "--extra-button=Cancel",
                        "--extra-button=No", "--extra-button=Yes"});
        break;
      case choice::retry_cancel:
        command.insert(command.end(),
                       {"--question", "--switch", "--extra-button=Cancel",
                        "--extra-button=Retry"});
        break;
      case choice::abort_retry_ignore:
        command.insert(command.end(),
                       {"--question", "--switch", "--extra-button=Ignore",
                        "--extra-button=Abort", "--extra-button=Retry"});
        break;
      case choice::ok:
      default:
        switch (_icon) {
          case icon::error:
            command.push_back("--error");
            break;
          case icon::warning:
            command.push_back("--warning");
            break;
          default:
            command.push_back("--info");
            break;
        }
    }

    command.insert(
        command.end(),
        {"--title", title, "--width=300", "--height=0",  // sensible defaults
         "--text", text, "--icon-name=dialog-" + get_icon_name(_icon)});
  } else if (is_kdialog()) {
    if (_choice == choice::ok) {
      switch (_icon) {
        case icon::error:
          command.push_back("--error");
          break;
        case icon::warning:
          command.push_back("--sorry");
          break;
        default:
          command.push_back("--msgbox");
          break;
      }
    } else {
      std::string flag = "--";
      if (_icon == icon::warning || _icon == icon::error)
        flag += "warning";
      flag += "yesno";
      if (_choice == choice::yes_no_cancel)
        flag += "cancel";
      command.push_back(flag);
      if (_choice == choice::yes_no || _choice == choice::yes_no_cancel) {
        m_mappings[0] = button::yes;
        m_mappings[256] = button::no;
      }
    }

    command.push_back(text);
    command.push_back("--title");
    command.push_back(title);

    // Must be after the above part
    if (_choice == choice::ok_cancel)
      command.insert(command.end(),
                     {"--yes-label", "OK", "--no-label", "Cancel"});
  }

  if (flags(flag::is_verbose))
    std::cerr << command << std::endl;

  m_async->start_process(command);
}

inline button message::result() {
  int exit_code;
  auto ret = m_async->result(&exit_code);
  if (exit_code < 0 ||  // this means cancel
      internal::ends_with(ret, "Cancel\n"))
    return button::cancel;
  if (internal::ends_with(ret, "OK\n"))
    return button::ok;
  if (internal::ends_with(ret, "Yes\n"))
    return button::yes;
  if (internal::ends_with(ret, "No\n"))
    return button::no;
  if (internal::ends_with(ret, "Abort\n"))
    return button::abort;
  if (internal::ends_with(ret, "Retry\n"))
    return button::retry;
  if (internal::ends_with(ret, "Ignore\n"))
    return button::ignore;
  if (m_mappings.count(exit_code) != 0)
    return m_mappings[exit_code];
  return exit_code == 0 ? button::ok : button::cancel;
}

// open_file implementation

inline open_file::open_file(
    std::string const& title,
    std::string const& default_path /* = "" */,
    std::vector<std::string> const& filters /* = { "All Files", "*" } */,
    opt options /* = opt::none */)
    : file_dialog(type::open, title, default_path, filters, options) {}

inline open_file::open_file(std::string const& title,
                            std::string const& default_path,
                            std::vector<std::string> const& filters,
                            bool allow_multiselect)
    : open_file(title,
                default_path,
                filters,
                (allow_multiselect ? opt::multiselect : opt::none)) {}

inline std::vector<std::string> open_file::result() {
  return vector_result();
}

// save_file implementation

inline save_file::save_file(
    std::string const& title,
    std::string const& default_path /* = "" */,
    std::vector<std::string> const& filters /* = { "All Files", "*" } */,
    opt options /* = opt::none */)
    : file_dialog(type::save, title, default_path, filters, options) {}

inline save_file::save_file(std::string const& title,
                            std::string const& default_path,
                            std::vector<std::string> const& filters,
                            bool confirm_overwrite)
    : save_file(title,
                default_path,
                filters,
                (confirm_overwrite ? opt::none : opt::force_overwrite)) {}

inline std::string save_file::result() {
  return string_result();
}

// select_folder implementation

inline select_folder::select_folder(std::string const& title,
                                    std::string const& default_path /* = "" */,
                                    opt options /* = opt::none */)
    : file_dialog(type::folder, title, default_path, {}, options) {}

inline std::string select_folder::result() {
  return string_result();
}

#endif  // SKIP_IMPLEMENTATION

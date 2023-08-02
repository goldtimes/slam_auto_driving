#include "tools/ui/pangolin_window_impl.h"
#include <glog/logging.h>
#include <pangolin/display/default_font.h>
#include <string>
#include <thread>

namespace lh::ui {
using UL = std::unique_lock<std::mutex>;

bool PangolinWindowImpl::Init() {}
bool PangolinWindowImpl::DeInit() {}
void PangolinWindowImpl::DrawAll() {}

void PangolinWindowImpl::Render() {}

void PangolinWindowImpl::CreateDisplayLayout() {}

void PangolinWindowImpl::AllocateBuffer() {}
void PangolinWindowImpl::ReleaseBuffer() {}

}  // namespace lh::ui
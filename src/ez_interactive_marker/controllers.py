from interactive_markers.menu_handler import *

class ControllerBase(object):
    def __init__(self):
        pass

class GroupedCheckStateController(ControllerBase):
    def __init__(self, server, menu_handler):
        self._server = server
        self._menu_handler = menu_handler
        self._group_list = {}

    def add_group(self, group, entry):
        self._group_list[entry] = group
        self._menu_handler.setCheckState(entry, MenuHandler.UNCHECKED)

    def update_check_state(self, entry):
        state = self._menu_handler.getCheckState(entry)
        if state == MenuHandler.NO_CHECKBOX:
            return
        elif state == MenuHandler.CHECKED:
            self._menu_handler.setCheckState(entry, MenuHandler.UNCHECKED)
        else: # MenuHandler.UNCHECKED
            self._menu_handler.setCheckState(entry, MenuHandler.CHECKED)
            target_gr = self._group_list[entry]
            for k, v in self._group_list.items():
                if v == target_gr and k != entry:
                    self._menu_handler.setCheckState(k, MenuHandler.UNCHECKED)
        self._menu_handler.reApply(self._server)
        self._server.applyChanges()

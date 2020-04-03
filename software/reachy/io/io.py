class IO(object):
    def find_module(self, module_name):
        raise NotImplementedError

    def find_dxl(self, dxl_name, dxl_id):
        raise NotImplementedError

    def find_orbital_disks(self):
        raise NotImplementedError

    def close(self):
        raise NotImplementedError

class event_ex:
    def __init__(self, boat_input):
        self.b_obj = boat_input
    def funca(self):
        self.b_obj.func_sail(self)

class boat_ex:
    def __init__(self):
        self.eevee = event_ex(boat_ex)
    def func_sail(self):
        pass
    def func_event_type(self):
        self.eevee.funca()

b_out = boat_ex()
b_out.func_event_type()
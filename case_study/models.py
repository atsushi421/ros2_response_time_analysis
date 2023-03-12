import pycpa.model as model


class PBurstModel (model.PJdEventModel):
    def __init__(self, P, burstlen, dmin, name='min', **kwargs):
        J = burstlen * P
        super().__init__(P, J, dmin, name=name, **kwargs)

class C():
    def __init__(self, ps = 720, ds = 10):
            self.pixelSpan = ps
            self.distSpan = ds

envdata = C()

def setup(pixelSpan, distSpan):
    envdata.pixelSpan = pixelSpan
    envdata.distSpan = distSpan

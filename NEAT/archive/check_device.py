import torch
print('version:        ' + torch.__version__)
print('cuda available: ' + str(torch.cuda.is_available()))
print('current device: ' + str(torch.cuda.current_device()))
print('device[0]     : ' + str(torch.cuda.device(0)))
print('device count  : ' + str(torch.cuda.device_count()))
print('device name   : ' + torch.cuda.get_device_name(0))
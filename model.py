import torch

'''
UNeXt Model Definition
'''
class ConvNextDownBlock(torch.nn.Module):
    def __init__(self, chan_ct, expansion = 4, stride = 1):
        super().__init__()
        self.block = torch.nn.Sequential(
            torch.nn.Conv2d(in_channels = chan_ct // stride, out_channels = chan_ct, kernel_size = 7, stride = stride, groups = chan_ct // stride, padding = 3, bias = True),
            torch.nn.BatchNorm2d(chan_ct),
            torch.nn.Conv2d(in_channels = chan_ct, out_channels = chan_ct * expansion, kernel_size = 1, bias = True),
            torch.nn.GELU(),
            torch.nn.Conv2d(in_channels = chan_ct * expansion, out_channels = chan_ct, kernel_size = 1, bias = True),
        )

        if stride > 1:
            self.shortcut = torch.nn.Conv2d(in_channels = chan_ct // stride, out_channels = chan_ct, kernel_size = stride, stride = stride, groups = chan_ct // stride, bias = False)
        else:
            self.shortcut = torch.nn.Identity()

    def forward(self, x):
        return self.block(x) + self.shortcut(x)


class ConvNextUpBlock(torch.nn.Module):
    def __init__(self, chan_ct, expansion = 4, stride = 1):
        super().__init__()
        self.block = torch.nn.Sequential(
            torch.nn.Conv2d(in_channels = chan_ct * stride, out_channels = chan_ct, kernel_size = 7, stride = 1, groups = chan_ct, padding = 3, bias = True),
            torch.nn.BatchNorm2d(chan_ct),
            torch.nn.Conv2d(in_channels = chan_ct, out_channels = chan_ct * expansion, kernel_size = 1, bias = True),
            torch.nn.GELU(),
            torch.nn.Conv2d(in_channels = chan_ct * expansion, out_channels = chan_ct, kernel_size = 1, bias = True),
        )

        if stride > 1:
            self.shortcut = torch.nn.Conv2d(in_channels = chan_ct * stride, out_channels = chan_ct, kernel_size = 1, stride = 1, bias = False)
        else:
            self.shortcut = torch.nn.Identity()

    def forward(self, x):
        return self.block(x) + self.shortcut(x)

class UNetDownBlock(torch.nn.Module):
    def __init__(self, chan_ct, expansion = 4, dropout = 0, first = False):
        super().__init__()
        module_list = []
        # module_list.append(torch.nn.Dropout2d(p = dropout))
        if not first:
            module_list.append(ConvNextDownBlock(chan_ct, expansion, stride = 2))
        module_list.append(ConvNextDownBlock(chan_ct, expansion))

        self.block = torch.nn.ModuleList(module_list)

    def forward(self, x):
        z = x
        for module in self.block:
            z = module(z)
        return z

class UNetUpBlock(torch.nn.Module):
    def __init__(self, chan_ct, expansion = 4, dropout = 0):
        super().__init__()
        module_list = []
        module_list.append(torch.nn.ConvTranspose2d(in_channels = chan_ct * 2, out_channels = chan_ct, kernel_size = 2, stride = 2, bias = True))
        # module_list.append(torch.nn.Dropout2d(p = dropout))
        module_list.append(ConvNextUpBlock(chan_ct, expansion, 2))
        module_list.append(ConvNextUpBlock(chan_ct, expansion))

        self.block = torch.nn.ModuleList(module_list)

    def forward(self, x, mirror_x):
        z = self.block[0](x)
        # z = self.block[1](z)
        z = self.block[1](torch.concat([mirror_x, z], dim = -3))
        z = self.block[2](z)
        return z

class VeinU_Net(torch.nn.Module):
    def __init__(self, base_chan = 64, dropout = 0, levels = 5):
        super().__init__()
        self.stem = torch.nn.Conv2d(in_channels = 3, out_channels = base_chan, kernel_size = 7, stride = 1, padding = 3, bias = True)
        self.levels = levels
        self.norm1 = torch.nn.BatchNorm2d(base_chan)

        blocks = []
        chan = base_chan
        blocks.append(UNetDownBlock(chan, expansion = 1, dropout = dropout, first = True))
        # Downward blocks
        for i in range(levels - 1):
            chan *= 2
            # blocks.append(UNetDownBlock2(chan))
            expansion_factor = 1
            if i == levels - 2:
                expansion_factor = 4
            elif i == levels - 3:
                expansion_factor = 2
            # expansion_factor = 4
            blocks.append(UNetDownBlock(chan, expansion_factor, dropout))

        # Upward blocks
        for i in range(levels - 1):
            chan //= 2
            # blocks.append(UNetUpBlock2(chan))
            expansion = 1
            if i == 0:
                expansion = 4
            elif i == 1:
                expansion = 2
            # expansion = 4
            blocks.append(UNetUpBlock(chan, expansion, dropout))

        self.U = torch.nn.ModuleList(blocks)
        self.end = torch.nn.Conv2d(in_channels = chan, out_channels = 2, kernel_size = 1, bias = True)

    def forward(self, x):
        responses = []
        z = self.norm1(self.stem(x))
        for i, block in enumerate(self.U):
            if i < self.levels:
                z = block(z)
                if i < self.levels - 1:
                    responses.append(z)
            else:
                j = 2 * self.levels - 2 - i # on block 5, use block 3, 6 use 2, 7 use 1
                z = block(z, responses[j])
        z = self.end(z)
        return z


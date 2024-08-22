import torch
import torch.nn as nn
from torch.autograd import Variable
from torch.nn.init import kaiming_normal_, orthogonal_
import numpy as np
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

def conv(batchNorm, in_planes, out_planes, kernel_size=3, stride=1, dropout=0):
    if batchNorm:
        return nn.Sequential(
            nn.Conv2d(in_planes, out_planes, kernel_size=kernel_size, stride=stride, padding=(kernel_size-1)//2, bias=False),
            nn.BatchNorm2d(out_planes),
            nn.LeakyReLU(0.1, inplace=True),
            nn.Dropout(dropout)#, inplace=True)
        )
    else:
        return nn.Sequential(
            nn.Conv2d(in_planes, out_planes, kernel_size=kernel_size, stride=stride, padding=(kernel_size-1)//2, bias=True),
            nn.LeakyReLU(0.1, inplace=True),
            nn.Dropout(dropout)#, inplace=True)
        )
    


class DeepVO(nn.Module):
    def __init__(self, imsize1=64, imsize2=64, batchNorm=True):
        super(DeepVO,self).__init__()
        # CNN
        self.rnn_hidden_size = 1000
        self.batchNorm = batchNorm
        self.conv1   = conv(self.batchNorm,   6,   128, kernel_size=7, stride=2, dropout=0.5)
        self.conv2   = conv(self.batchNorm,  128,  256, kernel_size=5, stride=2, dropout=0.5)
        self.conv3   = conv(self.batchNorm, 256,  512, kernel_size=5, stride=2, dropout=0.5)
        self.conv3_1 = conv(self.batchNorm, 512,  128, kernel_size=3, stride=1, dropout=0.5)
        self.conv4   = conv(self.batchNorm, 128, 64, kernel_size=3, stride=2, dropout=0.5)
        # Comput the shape based on diff image size
        __tmp = Variable(torch.zeros(1, 6, imsize1, imsize2))
        __tmp = self.encode_image(__tmp)
        # print(int(np.prod(__tmp.size())))

        # RNN
        self.rnn = nn.LSTM(
                    input_size=int(np.prod(__tmp.size())), 
                    hidden_size=self.rnn_hidden_size, 
                    num_layers=2, 
                    dropout=0.3, 
                    batch_first=True)
        self.rnn_drop_out = nn.Dropout(0.3)
        self.linear = nn.Linear(in_features=self.rnn_hidden_size, out_features=3)

    def forward(self, x1,x2):

        # x: (batch, seq_len, channel, width, height)
        # stack_image
        x = torch.cat((x1, x2), dim=1)
        batch_size = x.size(0)
        seq_len = 1
        # CNN
        # print(x.shape)
        x = self.encode_image(x)
        x = x.view(batch_size, seq_len, -1)
        # print(x.shape)
        # RNN
        out, hc = self.rnn(x)
        out = self.rnn_drop_out(out)
        out = self.linear(out)
        return out
        
    def encode_image(self, x):
        out_conv2 = self.conv2(self.conv1(x))
        out_conv3 = self.conv3_1(self.conv3(out_conv2))
        out_conv4 = self.conv4(out_conv3)
        return out_conv4



if __name__ == "__main__":
    from torchsummary import summary

    model = DeepVO().to(device)
    print(torch.cuda.is_available())
    for i in range(10):
        x1 = torch.randn(1,3,64,64).to(device)
        x2 = torch.randn(1,3,64,64).to(device)
        out = model(x1,x2)

        print(out.shape)
        print(out)

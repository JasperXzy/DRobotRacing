## Step 1 修改优化后的 Attention 模块 

文件目录：`ultralytics/nn/modules/block.py`, `Attention`类的`forward`方法替换成以下内容。主要的优化点是去除了一些无用的数据搬运操作，同时将 Reduce 的维度变为 C 维度，对 BPU 更加友好, 目前可以将 BPU 吞吐量翻倍, 并且不需要重新训练模型。

注：建议保留好原本的`forward`方法，例如改一个其他的名字`forward_`，方便在训练的时候换回来

```python
class Attention(nn.Module):
	def forward(self, x: torch.Tensor) -> torch.Tensor:     
		"""
		Forward pass of the Attention module.

		Args:
			x (torch.Tensor): The input tensor.

		Returns:
			(torch.Tensor): The output tensor after self-attention.
		"""
		B, C, H, W = x.shape
		N = H * W
		qkv = self.qkv(x)
		q, k, v = qkv.view(B, self.num_heads, self.key_dim * 2 + self.head_dim, N).split(
			[self.key_dim, self.key_dim, self.head_dim], dim=2
		)
		
		attn = (q.transpose(-2, -1) @ k) * self.scale
		attn = attn.permute(0, 3, 1, 2).contiguous()  # CHW2HWC like
		max_attn = attn.max(dim=1, keepdim=True).values 
		exp_attn = torch.exp(attn - max_attn)
		sum_attn = exp_attn.sum(dim=1, keepdim=True)
		attn = exp_attn / sum_attn
		attn = attn.permute(0, 2, 3, 1).contiguous()  # HWC2CHW like
		x = (v @ attn.transpose(-2, -1)).view(B, C, H, W) + self.pe(v.reshape(B, C, H, W))
		x = self.proj(x)
		return x
```

## Step 2 修改 Detect 的输出头

直接将三个特征层的Bounding Box信息和Classify信息分开输出，一共6个输出头。

文件目录：`ultralytics/nn/modules/head.py`，`Detect`类的 `forward` 方法替换成以下内容。

注：建议保留好原本的`forward`方法，例如改一个其他的名字`forward_`, 方便在训练的时候换回来

```python
class Detect(nn.Module):
	def forward(self, x):
		"""Concatenate and return predicted bounding boxes and class probabilities."""
		result = [] 
		for i in range(self.nl):
			result.append(self.cv3[i](x[i]).permute(0, 2, 3, 1).contiguous())
			result.append(self.cv2[i](x[i]).permute(0, 2, 3, 1).contiguous())
		return result
```
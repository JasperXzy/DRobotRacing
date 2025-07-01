from openai import OpenAI
import os
import base64


def encode_image(image_path):
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode("utf-8")

base64_image = encode_image("target.jpg")

client = OpenAI(
    api_key="sk-283eeb7b6ee74671a6ebbfdcbf95303c",
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1",
)

completion = client.chat.completions.create(
    model="qwen-vl-plus",
    messages=[
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"},
                },
                {"type": "text", "text": "描述图中病人的信息以及其所处环境"},
            ],
        }
    ]
)

print(completion.choices[0].message.content)

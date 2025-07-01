import openai
import base64

openai.api_key = "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"

def encode_image(target_image_path):
    with open(target_image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

image_path = "./target.jpg"
base64_image = encode_image(image_path)

response = openai.chat.completions.create(
    model="gpt-4o",
    messages=[
        {
            "role": "user",
            "content": [
                {"type": "text", "text": "请识别这张图片并用中文描述图中病人的信息以及其所处环境"},
                {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}},
            ],
        }
    ],
    max_tokens=300,
)

print(response.choices[0].message.content)

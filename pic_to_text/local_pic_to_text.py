import requests
import base64

def encode_image_to_base64(image_path):
    with open(image_path, "rb") as img_file:
        return base64.b64encode(img_file.read()).decode('utf-8')

def generate_image_caption(image_path, api_url="http://127.0.0.1:1234/v1/chat/completions"):
    img_b64 = encode_image_to_base64(image_path)
    img_mime = "image/jpeg"

    headers = {"Content-Type": "application/json"}
    data = {
        "model": "google/gemma-3-4b",
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": "请描述这张图片中的病人信息以及其所处环境"},
                    {"type": "image_url",
                        "image_url": {"url": f"data:{img_mime};base64,{img_b64}"}
                    }
                ]
            }
        ],
        "max_tokens": 1024,
        "temperature": 0.5
    }

    response = requests.post(api_url, headers=headers, json=data)
    if response.status_code == 200:
        result = response.json()
        caption = result['choices'][0]['message']['content']
        return caption
    else:
        print("Error:", response.status_code, response.text)
        return None

if __name__ == "__main__":
    img_path = "target.jpg"
    description = generate_image_caption(img_path)
    print(description)

import alexa_like_whisper
import os
import openai

openai.api_key = os.getenv('OPENAI_API_KEY')


if __name__ == "__main__":
    # Modelsizes on whisper
    MODELSIZES = ['tiny', 'base', 'small', 'medium', 'large']
 
    # AccessKey obtained from Picovoice Console (https://console.picovoice.ai/)
    ACCESS_KEY = os.getenv('PICOVOICE_API_KEY')
    KEYWORD_PATH = ['robot_en_linux_v3_0_0.ppn']
 
    # Recording Time(s)
    RECORDING_TIME = 3
 
    # if using API, set True
    WHISPER_API = True
 
    alexa_like = alexa_like_whisper.AlexaLikeWhisper(ACCESS_KEY, KEYWORD_PATH, MODELSIZES[3], RECORDING_TIME, WHISPER_API)
 
    print('start')
    while True:
        result = alexa_like.run()
        if result == 'Sleep':
            None
        elif result == 'On recording...' or result == 'Wake':
            print(result)
        else:
            print(result)

            functions = [
                {
                    "name" : "goto",
                    "description": "move to the target location",
                    "parameters" : {
                        "type": "object",
                        "properties": {
                            "location": {
                                "type": "string",
                                "description": "target location to move, e.g. kitchen, room_a, room_b, room_c, entrance, living",
                            },
                        },
                        "required": ["location"],
                    },
                },
            ]

            my_messages=[
                {"role": "user", "content": result},
            ]

            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=my_messages,
                functions = functions,
                temperature=0.2,
                max_tokens=100
            )
            print(response)
            if response.choices[0].finish_reason == 'function_call':
                print('##############')
                print(response.choices[0].message.function_call)
                print('##############')


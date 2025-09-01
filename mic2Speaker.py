'''
(virtual_env)
$ python -m pip install PyAudio
'''

import pyaudio
import wave

CHUNK = 512

# PyAudioのインスタンスを作成
p = pyaudio.PyAudio()

# Bluetoothマイクからの音声ストリームを開く（デバイス名やインデックスを調整してください）
stream_in_out = p.open(format=pyaudio.paInt16, # サンプルフォーマット
            channels=1, # チャンネル数
            rate=16000, # サンプリングレート
            frames_per_buffer = CHUNK,
            input=True, # 入力ストリーム
            output=True, # 出力ストリーム
            #input_device_index=YOUR_INPUT_DEVICE_INDEX, # マイクのデバイスインデックス
            )

#stream_out = p.open(format=pyaudio.paInt16, # サンプルフォーマット
#            channels=1, # チャンネル数
#            rate=16000, # サンプリングレート
#            #input=True, # 入力ストリーム
#            output=True, # 出力ストリーム
#            #output_device_index=YOUR_OUTPUT_DEVICE_INDEX, # マイクのデバイスインデックス
#            )

while stream_in_out.is_active():
    #current = time.time()
    #end = time.time() + TIMEOUT_LENGTH

    data = stream_in_out.read(CHUNK)

    output = stream_in_out.write(data) # ここでスピーカーから音声を出力したい

stream_in_out.stop_stream()
stream_in_out.close()

p.terminate()
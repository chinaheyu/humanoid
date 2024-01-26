#/usr/bin/bash

set -e

curl --location --request POST "https://$AZURE_SPEECH_REGION.tts.speech.microsoft.com/cognitiveservices/v1" \
    --header "Ocp-Apim-Subscription-Key: $AZURE_SPEECH_KEY" \
    --header 'Content-Type: application/ssml+xml' \
    --header 'X-Microsoft-OutputFormat: audio-48khz-192kbitrate-mono-mp3' \
    --header 'User-Agent: curl' \
    --data-raw '<speak version="1.0" xmlns="http://www.w3.org/2001/10/synthesis" xmlns:mstts="https://www.w3.org/2001/mstts" xml:lang="zh-CN" >
    <voice name="zh-CN-XiaoxiaoNeural" >
        <mstts:express-as style="affectionate">'$2'</mstts:express-as>
    </voice>
</speak>' | ffmpeg -y -f mp3 -i - -acodec pcm_s16le -ar 16000 -ac 1 -f wav "$1.wav"

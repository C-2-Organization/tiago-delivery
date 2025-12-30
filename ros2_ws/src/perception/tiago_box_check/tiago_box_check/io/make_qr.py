import qrcode

text = "104호"   # QR에 넣을 텍스트(데이터)
img = qrcode.make(text)
img.save("qr_104.png")
print("saved: qr_104.png")

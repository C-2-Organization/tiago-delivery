import qrcode

for i in range(1301, 1309):
    text = f"{i}호"
    img = qrcode.make(text)
    img.save(f"qr_{i}.png")
    print(f"saved: qr_{i}.png")

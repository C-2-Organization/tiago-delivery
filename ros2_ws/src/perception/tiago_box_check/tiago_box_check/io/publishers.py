from std_msgs.msg import String

class OCRPublishers:
    def __init__(self, node):
        self.pub_text_raw = node.create_publisher(String, "/ocr/text_raw", 10)
        self.pub_dest_id  = node.create_publisher(String, "/delivery/destination_id", 10)

    def publish_text_raw(self, text: str):
        self.pub_text_raw.publish(String(data=text))

    def publish_destination_id(self, dest_id: str):
        self.pub_dest_id.publish(String(data=dest_id))

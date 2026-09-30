import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray


class Client(Node):
  def __init__( self):
    super().__init__( 'pose loader' )
    #Nodo comunica goal list
    self.goals_pub = self.create_publisher( PoseArray, 'goal_list', 10 )

def leer_txt(self):
    #Aquí hay que poner la lógica real para leer el mensaje
    msg_goal_list = PoseArray() 
    
    self.goals_pub.publish( msg_goal_list )


def main(args=None):
  rclpy.init()
  N = Client()

  #LLamamos para leer el txt
  N.leer_txt()
  rclpy.spin(N)


if __name__ == '__main__':
  main()

import sys
import tty
import termios

def getch():
    """
    Get single character from standard input stream
    """

	fd = sys.stdin.fileno()
	old_settings = termios.tcgetattr(fd)

	try:
		tty.setraw(sys.stdin.fileno())
		ch = sys.stdin.read(1)
		print(ch)
	except:
		print("error")
	finally:
		termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

	return ch
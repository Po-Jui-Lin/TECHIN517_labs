#!/usr/bin/env python
import rospy
import sys
from pbd_recorder import PbdRecorder

def main():
    rospy.init_node('pbd_cli')
    recorder = PbdRecorder()
    
    print("\n=== PbD Command Line Interface ===")
    print("Commands:")
    print("  create <name> - Create new program")
    print("  save base - Save pose relative to base")
    print("  save marker <id> - Save pose relative to marker")
    print("  open - Open gripper")
    print("  close - Close gripper")
    print("  save_program <filename> - Save program to file")
    print("  quit - Exit")
    
    while not rospy.is_shutdown():
        try:
            cmd = input("\nPbD> ").strip().split()
            
            if not cmd:
                continue
                
            if cmd[0] == 'create':
                if len(cmd) > 1:
                    recorder.create_program(cmd[1])
                else:
                    print("Usage: create <name>")
                    
            elif cmd[0] == 'save':
                if len(cmd) > 1 and cmd[1] == 'base':
                    recorder.save_pose('base')
                elif len(cmd) > 2 and cmd[1] == 'marker':
                    recorder.save_pose('marker', int(cmd[2]))
                else:
                    print("Usage: save base | save marker <id>")
                    
            elif cmd[0] == 'open':
                recorder.set_gripper('open')
                
            elif cmd[0] == 'close':
                recorder.set_gripper('close')
                
            elif cmd[0] == 'save_program':
                if len(cmd) > 1:
                    recorder.save_program(cmd[1])
                else:
                    print("Usage: save_program <filename>")
                    
            elif cmd[0] == 'quit':
                break
                
            else:
                print("Unknown command: {}".format(cmd[0]))
                
        except KeyboardInterrupt:
            break
    
    print("\nExiting...")

if __name__ == '__main__':
    main()
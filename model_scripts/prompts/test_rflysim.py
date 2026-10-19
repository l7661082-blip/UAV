import re
import os
from transformers import AutoModel, AutoTokenizer
import torch
import rospy
from std_msgs.msg import String

class Multi_RflySim:
    def __init__(self, prompt=None):
        # 默认使用任务提示词
        if prompt is None:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            prompt = os.path.join(script_dir, "prompts", "mission_prompt.txt")
        
        knowledge_prompt = open(prompt, "r", encoding="utf-8").read()
        
        # 加载本地大模型 FM9G4B-V
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.model_file = os.path.join(script_dir, 'FM9G4B-V')
        
        rospy.loginfo("Loading model from %s ...", self.model_file)
        self.model = AutoModel.from_pretrained(
            self.model_file, trust_remote_code=True,
            attn_implementation='sdpa', torch_dtype=torch.bfloat16
        )
        self.model = self.model.eval().cuda()
        self.tokenizer = AutoTokenizer.from_pretrained(
            self.model_file, trust_remote_code=True
        )
        rospy.loginfo("Model loaded successfully.")
        
        # 聊天历史（多轮对话）
        self.msgs = []
        # 通过聊天历史注入知识库（提示词）
        self.ask(knowledge_prompt)

    def ask(self, prompt):
        self.msgs.append(
            {"role": "user", "content": [prompt]},
        )
        result = self.model.chat(
            image=None,
            msgs=self.msgs,
            tokenizer=self.tokenizer,
            sampling=True,
            stream=True,
        )
        print("\n", "=" * 100, "\n")
        content = ""
        for new_text in result:
            print(new_text, flush=True, end='')
            content += new_text

        # 保存历史记录（多轮对话）
        self.msgs.append(
            {
                "role": "assistant",
                "content": [content],
            }
        )

        return self.msgs[-1]["content"]

    def extract_code(self, content):
        """
        从 LLM 回复中提取 ```command ... ``` 代码块中的指令
        返回指令列表（每行一条）
        """
        code_block_regex = re.compile(r"```(.*?)```", re.DOTALL)
        code_blocks = code_block_regex.findall(content[0])
        if code_blocks:
            full_code = "\n".join(code_blocks)
            # 去除 "command" 标记
            if full_code.startswith("command"):
                full_code = full_code[len("command"):]
            return full_code.strip()
        else:
            return None

    def process(self, command):
        """
        处理用户输入：
        1. 发送给 LLM
        2. 从回复中提取指令
        3. 返回指令字符串（每行一条指令）
        """
        response = self.ask(command)
        code = self.extract_code(response)
        print("\n\n[提取的指令]:", code)
        if code:
            return code
        else:
            rospy.logwarn("LLM 未返回有效指令，原始回复: %s", response)
            return None


def result_callback(msg):
    """任务执行结果回调"""
    rospy.loginfo("[任务结果] %s", msg.data)


if __name__ == "__main__":
    rospy.init_node('mission_commander')
    
    # 发布指令到 /mission_command（bt_ros LLM 模式监听此 topic）
    pub = rospy.Publisher('/mission_command', String, queue_size=10)
    # 中止指令发布者
    abort_pub = rospy.Publisher('/mission_abort', String, queue_size=10)
    # 订阅执行结果
    rospy.Subscriber('/mission_result', String, result_callback)
    
    # 等待 publisher 注册完成
    rospy.sleep(1.0)
    
    # 初始化大模型
    rospy.loginfo("正在加载大模型，请稍候...")
    multi_rflysim = Multi_RflySim()
    rospy.loginfo("大模型加载完成，等待输入指令。")
    
    print("\n" + "=" * 60)
    print(" 大模型无人机任务控制系统")
    print(" - 输入自然语言指令控制无人机")
    print(" - 输入 '退出' 结束程序")
    print(" - 输入 '中止' 取消正在执行的任务")
    print("=" * 60 + "\n")
    
    while not rospy.is_shutdown():
        try:
            command = input("请输入命令: ")
        except EOFError:
            break
            
        if command == "退出":
            break
        
        if command == "中止":
            abort_pub.publish(String(data="abort"))
            rospy.loginfo("已发送中止指令。")
            continue
        
        if not command.strip():
            continue
        
        mission = multi_rflysim.process(command)
        if mission:
            print(f"\n[发送指令序列]\n{mission}\n")
            pub.publish(String(data=mission))
            rospy.loginfo("指令已发送到 /mission_command")
        else:
            print("\n[警告] 大模型未返回有效指令，请重新输入。\n")

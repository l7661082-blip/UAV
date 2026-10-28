"""
一次性任务生成脚本 —— 输入完整的自然语言任务描述，大模型一次性生成所有指令并发送

用法:
    python3 mission_once.py "起飞，穿过框，识别二维码，再穿回来，去A区打红色气球，降落"
    python3 mission_once.py  (交互式输入)
"""
import sys
import os
import re
import hashlib
from transformers import AutoModel, AutoTokenizer
import torch
import rospy
from std_msgs.msg import String

SIMPLE_COMMANDS = {
    "takeoff", "cross_frame", "detect_aruco", "return_cross_frame", "land"
}
BALLOON_COMMAND = re.compile(r"hit_balloon:(red|purple|blue)\Z")
COMMAND_BLOCK = re.compile(r"```command[ \t]*\r?\n(.*?)\r?\n```", re.DOTALL)


def parse_model_commands(content):
    """只接受行为树文本入口支持的命令，避免执行残缺任务。"""
    blocks = COMMAND_BLOCK.findall(content)
    if len(blocks) != 1:
        raise ValueError("模型回复必须恰有一个 command 代码块")
    commands = [line.strip() for line in blocks[0].splitlines() if line.strip()]
    if not commands:
        raise ValueError("模型回复的命令列表为空")
    invalid = [cmd for cmd in commands
               if cmd not in SIMPLE_COMMANDS and not BALLOON_COMMAND.fullmatch(cmd)]
    if invalid:
        raise ValueError("不支持的命令: " + ", ".join(invalid))
    return "\n".join(commands)


def load_model():
    """加载本地大模型"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    prompt_path = os.path.join(script_dir, "prompts", "mission_prompt.txt")
    model_dir = os.path.join(script_dir, "FM9G4B-V")

    with open(prompt_path, "rb") as prompt_file:
        prompt_bytes = prompt_file.read()
    knowledge_prompt = prompt_bytes.decode("utf-8")
    rospy.loginfo("模型脚本: %s", os.path.realpath(__file__))
    rospy.loginfo("模型目录: %s", os.path.realpath(model_dir))
    rospy.loginfo("提示词: %s SHA-256=%s", os.path.realpath(prompt_path),
                  hashlib.sha256(prompt_bytes).hexdigest())

    print("[1/3] 加载模型...", flush=True)
    model = AutoModel.from_pretrained(
        model_dir, trust_remote_code=True,
        attn_implementation='sdpa', torch_dtype=torch.bfloat16
    ).eval().cuda()
    tokenizer = AutoTokenizer.from_pretrained(model_dir, trust_remote_code=True)

    # 注入提示词
    print("[2/3] 注入提示词...", flush=True)
    msgs = [{"role": "user", "content": [knowledge_prompt]}]
    result = model.chat(image=None, msgs=msgs, tokenizer=tokenizer, sampling=True, stream=True)
    reply = ""
    for t in result:
        reply += t
    msgs.append({"role": "assistant", "content": [reply]})

    print("[3/3] 提示词注入完成，模型就绪。\n", flush=True)
    return model, tokenizer, msgs


def generate_commands(model, tokenizer, base_msgs, task_description):
    """向大模型发送任务描述，提取指令列表（每次只保留初始提示词，不累积历史）"""
    # 每次新建对话：初始提示词 + 本次任务
    msgs = base_msgs.copy()
    msgs.append({"role": "user", "content": [task_description]})

    result = model.chat(image=None, msgs=msgs, tokenizer=tokenizer, sampling=True, stream=True)
    content = ""
    for t in result:
        print(t, flush=True, end="")
        content += t
    print()

    # 正常推理后保留 CUDA 缓存，避免下一次推理重新分配显存。
    return parse_model_commands(content)


def main():
    rospy.init_node("mission_once", anonymous=True)
    pub = rospy.Publisher("/mission_command", String, queue_size=10)
    ready_pub = rospy.Publisher("/mission_llm_ready", String, queue_size=1, latch=True)

    model, tokenizer, base_msgs = load_model()
    ready_pub.publish(String(data="READY"))
    rospy.loginfo("/mission_llm_ready: READY")

    def publish_task(task):
        try:
            commands = generate_commands(model, tokenizer, base_msgs, task)
        except ValueError as e:
            rospy.logerr("拒绝模型输出，任务未发布: %s", e)
            return
        except RuntimeError as e:
            rospy.logerr("推理失败，任务未发布: %s", e)
            torch.cuda.empty_cache()
            return
        rospy.loginfo("通过校验的命令序列:\n%s", commands)
        pub.publish(String(data=commands))
        rospy.loginfo("指令已发送到 /mission_command")

    # 如果命令行有参数，先执行一次
    if len(sys.argv) > 1:
        task = " ".join(sys.argv[1:])
        print(f"\n[任务] {task}")
        print("=" * 60)
        publish_task(task)

    # 持续等待用户输入（方便反复调试）
    while not rospy.is_shutdown():
        print("\n" + "=" * 60)
        print("请输入任务描述（输入 quit 退出）：")
        try:
            task = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出。")
            break
        if not task:
            continue
        if task.lower() in ("quit", "exit", "q"):
            print("退出。")
            break

        print(f"\n[任务] {task}")
        print("=" * 60)

        publish_task(task)


if __name__ == "__main__":
    main()

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
import time
from transformers import AutoModel, AutoTokenizer
import torch
import rospy
from std_msgs.msg import String

SIMPLE_COMMANDS = {
    "takeoff", "cross_frame", "detect_aruco", "return_cross_frame", "land"
}
BALLOON_COMMAND = re.compile(r"hit_balloon:(red|purple|blue)\Z")
FENCED_BLOCK = re.compile(r"```([^\n]*)\n(.*?)\n```", re.DOTALL)


def parse_model_commands(content):
    """只接受行为树文本入口支持的命令，避免执行残缺任务。"""
    # 模型流可能只用 CR 分隔行，先统一整个回复以便定位代码块。
    content = content.replace("\r\n", "\n").replace("\r", "\n")
    blocks = FENCED_BLOCK.findall(content)
    if len(blocks) > 1:
        raise ValueError("模型回复包含多个代码块，疑似复述提示词示例，拒绝执行")
    if not blocks:
        # 无围栏降级：只认"整行去掉空白后恰好是白名单命令"的行，
        # 中文解释行不可能整行等于命令名，不会误入。
        no_fence = [line.strip() for line in content.split("\n")
                    if line.strip() in SIMPLE_COMMANDS
                    or BALLOON_COMMAND.fullmatch(line.strip())]
        if not no_fence:
            raise ValueError("模型回复中既无代码块也无白名单命令行")
        rospy.logwarn("模型回复未使用代码块，按整行白名单降级提取")
        return "\n".join(no_fence)
    command_blocks = [code for label, code in blocks if label.strip() == "command"]
    # 多块回复已拒绝；单块时优先取 command，缺失标签才降级。
    if command_blocks:
        code = command_blocks[0]
    else:
        code = blocks[0][1]
        rospy.logwarn("模型回复使用非 command 代码块，按命令白名单降级校验")
    code = code.replace("\r\n", "\n").replace("\r", "\n").strip()
    if not code:
        raise ValueError("模型回复的命令列表为空")
    # 空行不是命令，必须丢弃：模型常在命令块里插入空行，若当命令校验会误拒整段任务
    commands = [line.strip() for line in code.split("\n") if line.strip()]
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
    try:
        return parse_model_commands(content)
    except ValueError as e:
        # 拒绝时把原始回复带回日志，方便定位模型到底输出了什么格式
        raise ValueError(f"{e}；模型原始回复(前300字符)={content[:300]!r}") from None


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
        deadline = time.monotonic() + 120.0
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            if time.monotonic() >= deadline:
                rospy.logerr("120 秒内没有 /mission_command 接收者，任务未发布")
                return
            time.sleep(0.2)
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

#ifndef SIMPLE_SERIAL_SHELL_HPP
#define SIMPLE_SERIAL_SHELL_HPP

#ifndef SIMPLE_SERIAL_SHELL_BUFSIZE
#define SIMPLE_SERIAL_SHELL_BUFSIZE 88
#endif

#include <Arduino.h>

/*!
 *  @file SimpleSerialShell.hpp
 *
 *  @section dependencies 依赖项
 *
 *  依赖于 Stream。该 shell 是 Stream 的一个实例，因此与 Stream 兼容的任何内容也应该与 shell 一起工作。
 *
 *  @section author 作者
 *  Phil Jansen
 */
class SimpleSerialShell : public Stream {
public:
    // Shell 的单例实例
    static SimpleSerialShell theShell;

    // Unix 风格（从 1970 年起！）
    // 函数必须具有类似“int hello(int argc, char ** argv)”的签名
    typedef int (*CommandFunction)(int, char **);

    /**
     * @brief 注册命令到 shell 处理器。
     * 
     * @param name 命令名称，可选文档说明。命令必须通过一个空格与文档说明分隔，这意味着命令本身不能包含空格字符。空格后的部分会作为文档显示在帮助消息中。
     * @param f 当输入命令时调用的命令函数。
     */
    void addCommand(const __FlashStringHelper * name, CommandFunction f);

    /**
     * @brief 将 shell 连接到一个流对象。
     * 
     * @param shellSource 连接的流对象。
     */
    void attach(Stream & shellSource);

    /**
     * @brief 检查是否有完整的命令，并在可用时执行。非阻塞。
     * 
     * @return 如果尝试执行命令返回 true。
     */
    bool executeIfInput(void);

    /**
     * @brief 获取最近的错误代码。
     * 
     * @return 最近的错误代码。
     */
    int lastErrNo(void);

    /**
     * @brief 执行命令字符串。
     * 
     * @param aCommandString 要执行的命令字符串。
     * @return 执行结果。
     */
    int execute(const char aCommandString[]);

    /**
     * @brief 打印帮助信息。
     * 
     * @param argc 参数数量。
     * @param argv 参数列表。
     * @return 执行结果。
     */
    static int printHelp(int argc, char **argv);

    /**
     * @brief 重置命令输入缓冲区。
     */
    void resetBuffer(void);

    // 重写 Stream 类的函数，处理输入输出。
    virtual size_t write(uint8_t);
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual void flush(); // esp32 需要实现该函数

    typedef char* (*TokenizerFunction)(char* str, const char* delim, char** saveptr);

    /**
     * @brief 设置自定义的令牌分隔函数。
     * 
     * @param f 自定义分隔函数。
     */
    void setTokenizer(TokenizerFunction f);

private:
    SimpleSerialShell(void);

    Stream * shellConnection;
    int m_lastErrNo;
    int execute(void);
    int execute(int argc, char** argv);

    bool prepInput(void);

    int report(const __FlashStringHelper * message, int errorCode);
    static const char MAXARGS = 10;
    char linebuffer[SIMPLE_SERIAL_SHELL_BUFSIZE];
    int inptr;

    class Command;
    static Command * firstCommand;

    TokenizerFunction tokenizer;
};

/**
 * @brief 命令类，表示一个命令及其相关信息。
 */
class SimpleSerialShell::Command {
public:
    Command(const __FlashStringHelper * n, CommandFunction f)
        : nameAndDocs(n), myFunc(f) {}

    /**
     * @brief 执行命令。
     * 
     * @param argc 参数数量。
     * @param argv 参数列表。
     * @return 执行结果。
     */
    int execute(int argc, char **argv) {
        return myFunc(argc, argv);
    }

    /**
     * @brief 比较两个命令的名称。
     * 
     * @param other 另一个命令。
     * @return 比较结果。
     */
    int compare(const Command * other) const {
        const String otherNameString(other->nameAndDocs);
        return compareName(otherNameString.c_str());
    }

    /**
     * @brief 比较命令名称。
     * 
     * @param aName 需要比较的命令名称。
     * @return 比较结果。
     */
    int compareName(const char * aName) const {
        String work(nameAndDocs);
        int delim = work.indexOf(' ');
        if (delim >= 0) {
            work.remove(delim);
        }
        return strncasecmp(work.c_str(), aName, SIMPLE_SERIAL_SHELL_BUFSIZE);
    }

    /**
     * @brief 渲染命令的文档。
     * 
     * @param str 输出流。
     */
    void renderDocumentation(Stream& str) const {
        str.print(F("  "));
        str.print(nameAndDocs);
        str.println();
    }

    Command * next;

private:
    const __FlashStringHelper * const nameAndDocs;
    const CommandFunction myFunc;
};

// 全局命令实例
extern SimpleSerialShell& shell;



// Implementation

SimpleSerialShell SimpleSerialShell::theShell;
SimpleSerialShell& shell = SimpleSerialShell::theShell;
SimpleSerialShell::Command * SimpleSerialShell::firstCommand = NULL;

SimpleSerialShell::SimpleSerialShell()
    : shellConnection(NULL),
      m_lastErrNo(EXIT_SUCCESS),
      tokenizer(strtok_r) {
    resetBuffer();
    addCommand(F("help"), SimpleSerialShell::printHelp);
}

void SimpleSerialShell::addCommand(const __FlashStringHelper * name, CommandFunction f) {
    auto * newCmd = new Command(name, f);
    Command* temp2 = firstCommand;
    Command** temp3 = &firstCommand;
    while (temp2 != NULL && (newCmd->compare(temp2) > 0)) {
        temp3 = &temp2->next;
        temp2 = temp2->next;
    }
    *temp3 = newCmd;
    newCmd->next = temp2;
}

bool SimpleSerialShell::executeIfInput(void) {
    bool bufferReady = prepInput();
    bool didSomething = false;
    if (bufferReady) {
        didSomething = true;
        execute();
        print(F("> "));
    }
    return didSomething;
}

void SimpleSerialShell::attach(Stream & requester) {
    shellConnection = &requester;
}

bool SimpleSerialShell::prepInput(void) {
    bool bufferReady = false;
    bool moreData = true;
    do {
        int c = read();
        switch (c) {
            case -1:
                moreData = false;
                break;
            case 0:
                break;
            case 127: 
            case '\b':
                if (inptr > 0) {
                    print(F("\b \b"));
                    linebuffer[--inptr] = 0;
                }
                break;
            case 0x12: 
                print(F("\r\n"));
                print(linebuffer);
                break;
            case 0x15: 
                println(F("XXX"));
                resetBuffer();
                break;
            case ';':
            case '\r':
                println();
                bufferReady = true;
                break;
            case '\n':
                break;
            default:
                linebuffer[inptr++] = c;
                write(c);
                if (inptr >= SIMPLE_SERIAL_SHELL_BUFSIZE-1) {
                    bufferReady = true;
                }
                break;
        }
    } while (moreData && !bufferReady);
    return bufferReady;
}

int SimpleSerialShell::execute(const char commandString[]) {
    strncpy(linebuffer, commandString, SIMPLE_SERIAL_SHELL_BUFSIZE);
    return execute();
}

int SimpleSerialShell::execute(void) {
    char * argv[MAXARGS] = {0};
    linebuffer[SIMPLE_SERIAL_SHELL_BUFSIZE - 1] = '\0';
    int argc = 0;
    char * rest = NULL;
    const char * whitespace = " \t\r\n";
    char * commandName = tokenizer(linebuffer, whitespace, &rest);

    if (!commandName) {
        println(F("请输入 help 查看所有命令"));
        resetBuffer();
        return EXIT_SUCCESS;
    }
    argv[argc++] = commandName;

    for (; argc < MAXARGS;) {
        char * anArg = tokenizer(0, whitespace, &rest);
        if (anArg) {
            argv[argc++] = anArg;
        } else {
            return execute(argc, argv);
        }
    }

    return report(F("参数过多"), -1);
}

int SimpleSerialShell::execute(int argc, char **argv) {
    m_lastErrNo = 0;
    for (Command * aCmd = firstCommand; aCmd != NULL; aCmd = aCmd->next) {
        if (aCmd->compareName(argv[0]) == 0) {
            m_lastErrNo = aCmd->execute(argc, argv);
            resetBuffer();
            return m_lastErrNo;
        }
    }
    print(F("\""));
    print(argv[0]);
    print(F("\"："));
    return report(F("命令未找到 ,请输入 help 查看所有命令"), -1);
}

int SimpleSerialShell::lastErrNo(void) {
    return m_lastErrNo;
}

int SimpleSerialShell::report(const __FlashStringHelper * constMsg, int errorCode) {
    if (errorCode != EXIT_SUCCESS) {
        String message(constMsg);
        print(errorCode);
        if (message[0] != '\0') {
            print(F(": "));
            println(message);
        }
    }
    resetBuffer();
    m_lastErrNo = errorCode;
    return errorCode;
}

void SimpleSerialShell::resetBuffer(void) {
    memset(linebuffer, 0, sizeof(linebuffer));
    inptr = 0;
}

int SimpleSerialShell::printHelp(int /*argc*/, char ** /*argv*/) {
    shell.println(F("可用的命令如下："));
    auto aCmd = firstCommand;
    while (aCmd) {
        aCmd->renderDocumentation(shell);
        aCmd = aCmd->next;
    }
    return 0;
}

size_t SimpleSerialShell::write(uint8_t aByte) {
    return shellConnection ? shellConnection->write(aByte) : 0;
}

int SimpleSerialShell::available() {
    return shellConnection ? shellConnection->available() : 0;
}

int SimpleSerialShell::read() {
    return shellConnection ? shellConnection->read() : 0;
}

int SimpleSerialShell::peek() {
    return shellConnection ? shellConnection->peek() : 0;
}

void SimpleSerialShell::flush() {
    if (shellConnection) shellConnection->flush();
}

void SimpleSerialShell::setTokenizer(TokenizerFunction f) {
    tokenizer = f;
}

#endif /* SIMPLE_SERIAL_SHELL_HPP */
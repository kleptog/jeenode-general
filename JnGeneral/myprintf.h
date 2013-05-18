static void myputc(char ch)
{
    if(ch == '\n')
        Serial.write('\r');
    Serial.write(ch);
}

static void myputs(char *s)
{
    while(*s)
        myputc(*s++);
}

static void myputuint(unsigned int num, unsigned char base)
{
    if(num >= base)
        myputuint(num/base, base);
    char dig = num%base;
    dig = dig+(dig<10 ? '0' : 'A'-10);
    myputc(dig);
}

static void myputint(int num, unsigned char base)
{
    if (num<0) {
        num=-num;
        myputc('-');
    }
    myputuint(num,base);
}

static int myprintf(char *format, ...)
{
    va_list va;
    va_start(va,format);
    char ch;

    while ((ch=*(format++))) {
        if (ch!='%')  {
            myputc(ch);
            continue;
        }
        ch=*(format++);
        switch (ch) {
            case 0:
                goto abort;
            case 'u' :
                myputuint(va_arg(va, unsigned int),10);
                break;
            case 'd' :
                myputint(va_arg(va, int),10);
                break;
            case 'x': case 'X' :
                myputuint(va_arg(va, unsigned int),16);
                break;
            case 'c':
                myputc((char)(va_arg(va, int)));
                break;
            case 's':  {
                myputs(va_arg(va, char*));
                break;
            }
            case '%' :
                myputc(ch);
            default:
                break;
        }
    }
    abort:;
    va_end(va);
}

def t(n, f):
    try:
        print(n, 'OK', f())
    except BaseException as e:
        print(n, 'ERR', type(e).__name__, e)

setup()
start()
time.sleep(2)

try:
    t('frame noarg', lambda: pom.frame())
    t('frame robot', lambda: pom.frame('robot'))
    t('phynt desired', lambda: phynt.desired())
    t('wo data', lambda: phynt.get_wo_data())
    t('man desired', lambda: maneuver.desired())
    print('PHYNT', [a for a in dir(phynt) if not a.startswith('_')])
    print('POM', [a for a in dir(pom) if not a.startswith('_')])
finally:
    stop()
